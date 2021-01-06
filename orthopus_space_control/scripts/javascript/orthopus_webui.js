var ros;
var control_frame_sub;
var learning_mode_sub;
var hardware_status_sub;
var joystick_enable_sub;
var cmd_vel_pub;

var manage_position_srv;
var get_position_list_srv;

var GRIPPER_1 = 11;
var GRIPPER_2 = 12;
var GRIPPER_3 = 13;
var CMD_JOINTS = 1;
var CMD_TOOL = 6;
var OPEN_GRIPPER = 1;
var CLOSE_GRIPPER = 2;
var GRIPPER_SPEED = 300;

var pos_list;

/* Axis are map to a joy message of the XBOX controller */
var axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
var buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
var X_AXIS_INDEX = 4;
var Y_AXIS_INDEX = 0;
var Z_AXIS_INDEX = 1;
var ROTX_AXIS_INDEX = 7;
var ROTY_AXIS_INDEX = 6;
var ROTZ_AXIS_INDEX = 3;

var mainTimer;
var enable_loop = true;
var yz_joy;
var yx_joy;

var control_state = 0;
var learning_mode = false;
var joystick_enabled = false;
var hardware_status = {
  rpi_temperature: 0,
  hardware_version: 0,
  connection_up: false,
  error_message: "",
  calibration_needed: 0,
  calibration_in_progress: false,
  motor_names: [],
  motor_types: [],
  temperatures: [],
  voltages: [],
  hardware_errors: []
};

/* Converted from FsmInputEvent */
var INPUT_EVENT_NONE = 0;
var INPUT_EVENT_JOINT_POSITION = 1;
var INPUT_EVENT_SPACE_POSITION = 2;

function resetStatus() {
  hardware_status.rpi_temperature = 0;
  hardware_status.hardware_version = 0;
  hardware_status.connection_up = false;
  hardware_status.error_message = "";
  hardware_status.calibration_needed = 0;
  hardware_status.calibration_in_progress = false;
  hardware_status.motor_names = [];
  hardware_status.motor_types = [];
  hardware_status.temperatures = [];
  hardware_status.voltages = [];
  hardware_status.hardware_errors = [];
}

function stopLoop() {
  enable_loop = false;
  $('#status_spinner_color').removeClass();
  $('#status_spinner_color').addClass('text-danger');
  $('#status_spinner').removeClass();
  $('#status_spinner').addClass('spinner-grow spinner-grow-sm');
}

function UpdatePage() {
  /* Update status indicator */
  $('#status_spinner_color').removeClass();
  if (hardware_status.calibration_needed == 1) {
    $('#status_spinner_color').addClass('text-warning');
  }
  else if (hardware_status.connection_up == false) {
    $('#status_spinner_color').addClass('text-danger');
  }
  else {
    $('#status_spinner_color').addClass('text-success');
  }

  /* Update enable/disable indicator */
  if (learning_mode == true) {
    $('#enable_switch')[0].checked = false;
  }
  else {
    $('#enable_switch')[0].checked = true;
  }
  /* Update control frame switches indicator */
  if ((control_state & 0x01) != 0) {
    $('#position_frame_switch')[0].checked = true;
  }
  else {
    $('#position_frame_switch')[0].checked = false;
  }
  if ((control_state & 0x02) != 0) {
    $('#orientation_frame_switch')[0].checked = true;
  }
  else {
    $('#orientation_frame_switch')[0].checked = false;
  }
}

function Joystick(container, color, x_axis_remap, inv_x, y_axis_remap, inv_y, max_joy_size) {
  // joystck configuration, if you want to adjust joystick, refer to:
  // https://yoannmoinet.github.io/nipplejs/
  min_joy_size = 100;
  width = container.outerWidth();
  best_fit_width = Math.min(max_joy_size, width);
  console.log(best_fit_width);
  best_fit_width = Math.max(min_joy_size, best_fit_width)
  console.log(best_fit_width);
  container.height(best_fit_width);
  console.log(best_fit_width);
  joy_size = best_fit_width - 20;

  this.options = {
    zone: container[0],
    position: { left: '50%', top: '50%' },
    mode: 'static',
    size: joy_size,
    color: color,
    restJoystick: true
  };
  this.manager = nipplejs.create(options);

  // event listener for joystick move
  this.manager.on('move', function (evt, nipple) {
    var pix_max = (joy_size / 2.0);
    // Apply a square scale in order to increase small velocity control
    var dx = (Math.pow(nipple.distance * Math.cos(nipple.angle.radian), 2) / (1.0 * Math.pow(pix_max, 2))) * (inv_x ? (-1.0) : (1.0)) * Math.sign(Math.cos(nipple.angle.radian))
    var dy = (Math.pow(nipple.distance * Math.sin(nipple.angle.radian), 2) / (1.0 * Math.pow(pix_max, 2))) * (inv_y ? (-1.0) : (1.0)) * Math.sign(Math.sin(nipple.angle.radian))
    console.debug("dx : " + dx);
    console.debug("dy : " + dy);
    if (Math.abs(dx) < 0.005) {
      dx = 0.0;
    }
    if (Math.abs(dy) < 0.005) {
      dy = 0.0;
    }
    axes[x_axis_remap] = dx;
    axes[y_axis_remap] = dy;
  });
  // event litener for joystick release, always send stop message
  this.manager.on('end', function () {
    axes[x_axis_remap] = 0.0;
    axes[y_axis_remap] = 0.0;
  });
  return this.manager;
}

function mainLoop() {
  if (enable_loop) {
    publishCartesianVelocity();
  }
}

function refreshPositionList() {
  var request = new ROSLIB.ServiceRequest({});
  get_position_list_srv.callService(request, function (result) {
  pos_list = result.positions;
  // Populates position list 
  $('#dropdown_position_list')[0].innerHTML = '';
  $('#modal_position_list')[0].innerHTML = '';
  
  for(i in pos_list) {
    $('#dropdown_position_list')[0].innerHTML += '<a class="dropdown-item" href="#" id="id_go_' + i + '" title="' + pos_list[i].name + '" >' + pos_list[i].name + '</a>';
    var html_string = '\
        <div class="d-flex bd-highlight"> \
          <div class="p-2 flex-grow-1 bd-highlight">' + pos_list[i].name + '</div>';
    if(pos_list[i].name != "Home" && pos_list[i].name != "Rest")
    {
    html_string += '<button id="del_pos_' + i + '" value="' + pos_list[i].name + '" type="button" class="btn btn-outline-danger"> \
            <img src="delete.png" width="20"/>\
          </button>\
        </div>';
    }
    $('#modal_position_list')[0].innerHTML += html_string;
  }

  for(i in pos_list) {
      var id_go = '#id_go_' + i;
      $(id_go)
      .click(function () {
        console.log(this);       
        GoToPosition(this.title);
        });
      var id_delete = '#del_pos_' + i;
      $(id_delete)
        .click(function () {
          DeletePosition(this.value);
          });

  }

  });
}

function DeletePosition(pos_name)
{
  console.log('DeletePosition :  ' + pos_name);
  var request = new ROSLIB.ServiceRequest({
    cmd_type: 2,
    position_name: pos_name
  });

  manage_position_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + manage_position_srv.name
      + ': '
      + result.status
      + ', '
      + result.message);
      refreshPositionList();
  });
}

function AddPosition(pos_name)
{
  console.log('AddPosition :  ' + pos_name);
  var request = new ROSLIB.ServiceRequest({
    cmd_type: 0,
    position_name: pos_name
  });

  manage_position_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + manage_position_srv.name
      + ': '
      + result.status
      + ', '
      + result.message);
      refreshPositionList();
  });
}

function UpdatePosition(pos_name)
{
  console.log('UpdatePosition :  ' + pos_name);
  var request = new ROSLIB.ServiceRequest({
    cmd_type: 1,
    position_name: pos_name
  });

  manage_position_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + manage_position_srv.name
      + ': '
      + result.status
      + ', '
      + result.message);
      refreshPositionList();
  });
}

function publishCartesianVelocity() {
  var msg = new ROSLIB.Message({
    buttons: buttons,
    axes: axes
  });
  console.log(msg.axes);
  cmd_vel_pub.publish(msg);
}

function SetLearningMode(state) {
  console.log('SetLearningMode :  ' + state);

  var i = state ? 1 : 0;
  var request = new ROSLIB.ServiceRequest({
    value: i
  });

  var learning_mode_srv = new ROSLIB.Service({
    ros: ros,
    name: '/niryo_one/activate_learning_mode',
    serviceType: 'niryo_one_msgs/SetInt'
  });
  learning_mode_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + learning_mode_srv.name
      + ': '
      + result.status
      + ', '
      + result.message);
  });
}

function SetControlFrame() {
  var val = 0;
  var pos_state = $('#position_frame_switch')[0].checked;
  var i = pos_state ? 1 : 0;
  if (i == 1) {
    val = val | 0x01;
  }
  var orient_state = $('#orientation_frame_switch')[0].checked;
  var i = orient_state ? 1 : 0;
  if (i == 1) {
    val = val | 0x02;
  }
  console.log('SetOrientationControlFrame :  ' + val);

  var request = new ROSLIB.ServiceRequest({
    value: val
  });

  var set_control_frame_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_space_control/set_control_frame',
    serviceType: 'orthopus_space_control/SetUInt16'
  });
  set_control_frame_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + set_control_frame_srv.name
      + ': '
      + result.status
      + ', '
      + result.message);
  });
}

function Calibrate() {
  console.log('Robot calibration');

  var request = new ROSLIB.ServiceRequest({
    value: 2
  });

  var calib_srv = new ROSLIB.Service({
    ros: ros,
    name: '/niryo_one/calibrate_motors',
    serviceType: 'niryo_one_msgs/SetInt'
  });
  calib_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + calib_srv.name
      + ': '
      + result.status
      + ', '
      + result.message);
  });
}

function SetGripperId() {
  console.log('SetGripperId');

  var request = new ROSLIB.ServiceRequest({
    value: GRIPPER_2
  });

  var set_gripper_id_srv = new ROSLIB.Service({
    ros: ros,
    name: '/niryo_one/change_tool',
    serviceType: 'niryo_one_msgs/SetInt'
  });
  set_gripper_id_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + set_gripper_id_srv.name
      + ': '
      + result.state
    );
  });
}

function GripperOpen() {
  console.log('GripperOpen');

  var request = new ROSLIB.ServiceRequest({
    id: GRIPPER_2,
    open_position: 640,
    open_speed: 300,
    open_hold_torque: 128
  });

  var open_gripper_srv = new ROSLIB.Service({
    ros: ros,
    name: '/niryo_one/tools/open_gripper',
    serviceType: 'niryo_one_msgs/OpenGripper'
  });
  open_gripper_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + open_gripper_srv.name
      + ': '
      + result.state
    );
  });
}

function GripperClose() {
  console.log('GripperClose');

  var request = new ROSLIB.ServiceRequest({
    id: GRIPPER_2,
    close_position: 400,
    close_speed: 300,
    close_hold_torque: 128,
    close_max_torque: 1023
  });

  var open_gripper_srv = new ROSLIB.Service({
    ros: ros,
    name: '/niryo_one/tools/close_gripper',
    serviceType: 'niryo_one_msgs/CloseGripper'
  });
  open_gripper_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + open_gripper_srv.name
      + ': '
      + result.state
    );
  });
}

function GoToPosition(pos_name) {
  console.log('GoToPosition : ' + pos_name);

  var request = new ROSLIB.ServiceRequest({
    cmd_type: INPUT_EVENT_JOINT_POSITION,
    parameter: pos_name
  });

  var goto_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_space_control/set_robot_action',
    serviceType: 'niryo_one_msgs/SetRobotAction'
  });
  goto_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + goto_srv.name
      + ': '
      + result.status
      + ', '
      + result.message);
  });
}

function TakeDrink() {
  console.log('TakeDrink');

  var request = new ROSLIB.ServiceRequest({
    cmd_type: INPUT_EVENT_SPACE_POSITION,
    parameter: "TakeDrink"
  });

  var goto_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_space_control/set_robot_action',
    serviceType: 'niryo_one_msgs/SetRobotAction'
  });
  goto_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + goto_srv.name
      + ': '
      + result.status
      + ', '
      + result.message);
  });
}

function GiveDrink() {
  console.log('GiveDrink');

  var request = new ROSLIB.ServiceRequest({
    cmd_type: INPUT_EVENT_SPACE_POSITION,
    parameter: "GiveDrink"
  });

  var goto_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_space_control/set_robot_action',
    serviceType: 'niryo_one_msgs/SetRobotAction'
  });
  goto_srv.callService(request, function (result) {
    console.log('Result for service call on '
      + goto_srv.name
      + ': '
      + result.status
      + ', '
      + result.message);
  });
}

function RotationX(value) {
  axes[ROTX_AXIS_INDEX] = parseFloat(value);
}

function RotationY(value) {
  axes[ROTY_AXIS_INDEX] = parseFloat(value);
}

function RotationZ(value) {
  axes[ROTZ_AXIS_INDEX] = parseFloat(value);
}

window.onload = function () {
  var robot_IP = document.domain;
  if (robot_IP == "") {
    robot_IP = "localhost";
  }
  var ros_master_uri = "ws://" + robot_IP + ":9090";

  // Connecting to ROS
  // -----------------
  ros = new ROSLIB.Ros({
    url: ros_master_uri
  });

  ros.on('connection', function () {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
    resetStatus();
    stopLoop();
  });

  ros.on('close', function () {
    console.log('Connection to websocket server closed.');
    resetStatus();
    stopLoop();
  });

  // Setup topic subscribers
  // ----------------------
  learning_mode_sub = new ROSLIB.Topic({
    ros: ros,
    name: '/niryo_one/learning_mode',
    messageType: 'std_msgs/Bool'
  });
  learning_mode_sub.subscribe(function (message) {
    console.debug('Received message on ' + learning_mode_sub.name + ': ' + message.data);
    learning_mode = message.data;
    UpdatePage();
  });

  control_frame_sub = new ROSLIB.Topic({
    ros: ros,
    name: '/orthopus_space_control/control_feedback',
    messageType: 'std_msgs/UInt16'
  });
  control_frame_sub.subscribe(function (message) {
    console.debug('Received message on ' + control_frame_sub.name + ': ' + message.data);
    control_state = message.data;
    UpdatePage();
  });

  hardware_status_sub = new ROSLIB.Topic({
    ros: ros,
    name: '/niryo_one/hardware_status',
    messageType: 'niryo_one_msgs/HardwareStatus'
  });

  hardware_status_sub.subscribe(function (message) {
    console.debug('Received message on ' + hardware_status_sub.name + ': ');
    console.debug(message);
    hardware_status.rpi_temperature = message.rpi_temperature;
    hardware_status.hardware_version = message.hardware_version;
    hardware_status.connection_up = message.connection_up;
    hardware_status.error_message = message.error_message;
    hardware_status.calibration_needed = message.calibration_needed;
    hardware_status.calibration_in_progress = message.calibration_in_progress;
    hardware_status.motor_names = message.motor_names;
    hardware_status.motor_types = message.motor_types;
    hardware_status.temperatures = message.temperatures;
    hardware_status.voltages = message.voltages;
    hardware_status.hardware_errors = message.hardware_errors;

    UpdatePage();
  });

  joystick_enable_sub = new ROSLIB.Topic({
    ros: ros,
    name: '/niryo_one/joystick_interface/is_enabled',
    messageType: 'std_msgs/Bool'
  });

  joystick_enable_sub.subscribe(function (message) {
    console.debug('Received message on ' + joystick_enable_sub.name + ': ' + message.data);
    joystick_enabled = message.data;
    UpdatePage();
  });

  // Setup topic publishers 
  // ----------------------
  cmd_vel_pub = new ROSLIB.Topic({
    ros: ros,
    name: '/sim_joy',
    messageType: 'sensor_msgs/Joy'
  });

  // Setup service 
  // ----------------------
  manage_position_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_space_control/manage_position',
    serviceType: 'niryo_one_msgs/ManagePosition'
  });

  get_position_list_srv = new ROSLIB.Service({
    ros: ros,
    name: '/orthopus_space_control/get_position_list',
    serviceType: 'niryo_one_msgs/GetPositionList'
  });

  
  refreshPositionList();

  // Construct joystick objects 
  // ----------------------  
  var max_joy_size = $(window).height() - 420;
  yz_joy = Joystick($('#YZ_joystick'), '#0066ff', Y_AXIS_INDEX, true, Z_AXIS_INDEX, false, max_joy_size);
  yx_joy = Joystick($('#YX_joystick'), '#0066ff', Y_AXIS_INDEX, true, X_AXIS_INDEX, false, max_joy_size);

  // Populate video source 
  $('#video')[0].src = "http://" + robot_IP + ":8080/stream?topic=/image_raw";

  // Setup main loop timer
  // ----------------------
  mainTimer = setInterval(function () { mainLoop(); }, 100);

  SetGripperId();
}

window.onresize = function () {
  var max_joy_size = $(window).height() - 450;
  console.log("yz_joy")
  console.log(yz_joy)
  yz_joy.destroy();
  yx_joy.destroy();
  yz_joy = Joystick($('#YZ_joystick'), '#0066ff', Y_AXIS_INDEX, true, Z_AXIS_INDEX, false, max_joy_size);
  yx_joy = Joystick($('#YX_joystick'), '#0066ff', Y_AXIS_INDEX, true, X_AXIS_INDEX, false, max_joy_size);
}

$(function () {
  $('#enable_switch')
    .change(function () {
      var state = $('#enable_switch')[0].checked;
      SetLearningMode(!state);
    });
  $('#position_frame_switch')
    .change(function () {
      SetControlFrame();
    });
  $('#orientation_frame_switch')
    .change(function () {
      SetControlFrame();
    });

  $('#navbar')
    .click(function () {
      var state = $('#enable_switch')[0].checked;
      Calibrate();
    });


  $('#take_drink')
    .click(function () {
      TakeDrink();
    });
  $('#give_drink')
    .click(function () {
      GiveDrink();
    });
  $('#gripper_open')
    .click(function () {
      GripperOpen();
    });
  $('#gripper_close')
    .click(function () {
      GripperClose();
    });
  $('#rotX_slider')
    .on('input', function () {
      RotationX(this.value);
    })
    .change(function () {
      this.value = 0;
      RotationX(this.value);
    });
  $('#rotY_slider')
    .on('input', function () {
      RotationY(this.value);
    })
    .change(function () {
      this.value = 0;
      RotationY(this.value);
    });
  $('#rotZ_slider')
    .on('input', function () {
      RotationZ(this.value);
    })
    .change(function () {
      this.value = 0;
      RotationZ(this.value);
    });

  $('#addPositionForm')
    .on('submit', function (event) {
      filename = $('#new_position_name').val();
      console.log("filename = " + $('#new_position_name').val());
      // Check if entry exists
      for(i in pos_list) {
        if(pos_list[i].name == filename) {
          $('#addComment')[0].innerHTML = "";
          $('#overwriteModal').modal('toggle', filename);
          return;
        }
      }
      AddPosition(filename);
      $('#addComment').removeClass();
      $('#addComment').addClass("text-success font-italic text-center");
      $('#addComment')[0].innerHTML = "The position \""+filename+"\" has been added.";

    });
    
  $('#overwriteModal')
    .on('show.bs.modal', function (event) {
      var filename = event.relatedTarget;
      console.log(filename);
      $('#overwrite_pos').unbind();
      $('#overwrite_pos')
        .on('click', function () {
          UpdatePosition(filename);
          $('#addComment').removeClass();
          $('#addComment').addClass("text-warning font-italic text-center");          
          $('#addComment')[0].innerHTML = "The position \""+filename+"\" has been updated.";
        });
      
    });
});
