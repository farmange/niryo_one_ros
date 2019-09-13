var ros;
var learning_mode_sub;
var hardware_status_sub;
var joystick_enable_sub;
var cmd_vel_pub;
var PIX_LENGTH = 100.0;

var GRIPPER_1 = 11;
var GRIPPER_2 = 12;
var GRIPPER_3 = 13;
var CMD_JOINTS = 1;
var CMD_TOOL = 6;
var OPEN_GRIPPER = 1;
var CLOSE_GRIPPER = 2;
var GRIPPER_SPEED = 300;

var axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
var buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
var X_AXIS = 4;
var Y_AXIS = 0;
var Z_AXIS = 1;
var ROTX_AXIS = 7;
var ROTY_AXIS = 6;
var ROTZ_AXIS = 3;

var mainTimer;

var learning_mode = false;
var joystick_enabled = false;
var hardware_status = {
  rpi_temperature:0,
  hardware_version:0,
  connection_up:false,
  error_message:"",
  calibration_needed:0,
  calibration_in_progress:false,
  motor_names:[],
  motor_types:[],
  temperatures:[],
  voltages:[],
  hardware_errors:[]
};

var position_map = new Map();
position_map.set("Home", [0,0,0,0,0,0]);
position_map.set("Rest", [0,0.64,-1.397,0,0,0]);

function UpdatePage() {
  $('#enable_spinner_color').removeClass();
  if(hardware_status.calibration_needed == 1)
  {
    $('#enable_spinner_color').addClass('text-danger');
  }
  else if(hardware_status.connection_up == false)
  {
    $('#enable_spinner_color').addClass('text-warning'); 
  }
  else if(hardware_status.hardware_errors.length != 0)
  {
    $('#enable_spinner_color').addClass('text-primary');
  }
  else
  {
    $('#enable_spinner_color').addClass('text-success');
  }
  
  if(learning_mode == true)
  {
    $('#enable_switch')[0].checked = false;
  }
  else
  {
    $('#enable_switch')[0].checked = true;    
  }
    
  if(joystick_enabled == true)
  {
    $('#cartesian_switch')[0].checked = true;
  }
  else
  {
    $('#cartesian_switch')[0].checked = false;    
  }
}

function Joystick(container, color, x_axis_remap, inv_x, y_axis_remap, inv_y) {
      // joystck configuration, if you want to adjust joystick, refer to:
      // https://yoannmoinet.github.io/nipplejs/
      this.options = {
          zone: container,
          position: { left: '50%', top: '50%' },
          mode: 'static',
          size: PIX_LENGTH,
          color: color,
          restJoystick: true
      };
      this.manager = nipplejs.create(options);
      
      // Setup html container size
      container.style.height = PIX_LENGTH + 10 + "px";
      
      // event listener for joystick move
      this.manager.on('move', function (evt, nipple) {
          var pix_max = (PIX_LENGTH/2.0) ;
          // var pix_max = (PIX_LENGTH/2.0) * Math.cos(Math.PI/4); //
          // Apply a cubic scale in order to increase small velocity control
          var dx = (Math.pow(nipple.distance * Math.cos(nipple.angle.radian), 3) / Math.pow(pix_max, 3)) * (inv_x ?(-1.0):(1.0))
          var dy = (Math.pow(nipple.distance * Math.sin(nipple.angle.radian), 3) / Math.pow(pix_max, 3)) * (inv_y ?(-1.0):(1.0))
          console.debug("dx : " + dx)
          console.debug("dy : " + dy)
          axes[x_axis_remap] = dx;  
          axes[y_axis_remap] = dy;  
      });
      // event litener for joystick release, always send stop message
      this.manager.on('end', function () {
          axes[x_axis_remap] = 0.0;  
          axes[y_axis_remap] = 0.0;  
      });
      return this;
}

function mainLoop()
{
  publishCartesianVelocity();
}

function publishCartesianVelocity() {
  var msg = new ROSLIB.Message({
    buttons : buttons,
    axes : axes
  });
  console.log(msg.axes);
  cmd_vel_pub.publish(msg);
}

function SetLearningMode(state) {
  console.log('SetLearningMode :  ' + state);

  var i = state ? 1 : 0;
  var request = new ROSLIB.ServiceRequest({
    value : i
  });
  
  var learning_mode_srv = new ROSLIB.Service({
    ros : ros,
    name : '/niryo_one/activate_learning_mode',
    serviceType : 'niryo_one_msgs/SetInt'
  });
  learning_mode_srv.callService(request, function(result) {
  console.log('Result for service call on '
      + learning_mode_srv.name
      + ': '
      + result.status 
      + ', '
      + result.message);
  });
}
var ACTION_NONE             = 0; 
var ACTION_CARTESIAN        = 1;
var ACTION_GOTO_HOME        = 2;
var ACTION_GOTO_REST        = 3;
var ACTION_GOTO_DRINK       = 4;
var ACTION_FLIP_PINCH       = 5;

function SetJoystickEnable(state) {
  console.log('SetJoystickEnable');
  
  var request = new ROSLIB.ServiceRequest({
    value : ACTION_CARTESIAN
  });
  
  var joystick_enable_srv = new ROSLIB.Service({
    ros : ros,
    name : '/niryo_one/orthopus_interface/action',
    serviceType : 'niryo_one_msgs/SetInt'
  });
  joystick_enable_srv.callService(request, function(result) {
    console.log('Result for service call on '
      + joystick_enable_srv.name
      + ': '
      + result.status 
      + ', '
      + result.message);
  });
}


function GripperOpen() {
  console.log('GripperOpen');

  var request = new ROSLIB.ServiceRequest({
    id : GRIPPER_2,
    open_position : 640,
    open_speed : 300,
    open_hold_torque : 128
  });
  
  var open_gripper_srv = new ROSLIB.Service({
    ros : ros,
    name : '/niryo_one/tools/open_gripper',
    serviceType : 'niryo_one_msgs/OpenGripper'
  });
  open_gripper_srv.callService(request, function(result) {
    console.log('Result for service call on '
      + open_gripper_srv.name
      + ': '
      + result.state
    );
  });
//   GripperAction(GRIPPER_2, OPEN_GRIPPER)
}

function GripperClose() {
  console.log('GripperClose');

  var request = new ROSLIB.ServiceRequest({
    id : GRIPPER_2,
    close_position : 400,
    close_speed : 300,
    close_hold_torque : 128,
    close_max_torque : 1023
  });
  
  var open_gripper_srv = new ROSLIB.Service({
    ros : ros,
    name : '/niryo_one/tools/close_gripper',
    serviceType : 'niryo_one_msgs/CloseGripper'
  });
  open_gripper_srv.callService(request, function(result) {
    console.log('Result for service call on '
      + open_gripper_srv.name
      + ': '
      + result.state
    );
  });

//   GripperAction(GRIPPER_2, CLOSE_GRIPPER)
}

function GripperAction(id, action) {
//   console.log('Gripper : ' + ((action==1)?"Open":((action==2)?"Close":"Unknown action")));
//   
//   SetJoystickEnable(false);
//       
//   var robotActionClient = new ROSLIB.ActionClient({
//     ros : ros,
//     serverName : '/niryo_one/commander/robot_action',
//     actionName : 'niryo_one_msgs/RobotMoveAction'
//   });
// 
//   var goal = new ROSLIB.Goal({
//     actionClient : robotActionClient,
//     goalMessage : {
//       cmd : {
//         cmd_type : CMD_TOOL,
//         tool_cmd : {
//           tool_id : id,
//           cmd_type : action,
//           gripper_open_speed : GRIPPER_SPEED,
//         }
//       }
//     }
//   });
// 
//   goal.on('feedback', function(feedback) {
//     console.log('Feedback: ' + feedback.state);
//   });
// 
//   goal.on('result', function(result) {
//     console.log('Final Result: ' + result.message);
//   });
//   
//   goal.send();
}

function GotoHome() {
  console.log('GotoHome');
  
  var request = new ROSLIB.ServiceRequest({
    value : ACTION_GOTO_HOME
  });
  
  var goto_srv = new ROSLIB.Service({
    ros : ros,
    name : '/niryo_one/orthopus_interface/action',
    serviceType : 'niryo_one_msgs/SetInt'
  });
  goto_srv.callService(request, function(result) {
    console.log('Result for service call on '
      + goto_srv.name
      + ': '
      + result.status 
      + ', '
      + result.message);
  });
}

function GotoRest() {
  console.log('GotoRest');
  
  var request = new ROSLIB.ServiceRequest({
    value : ACTION_GOTO_REST
  });
  
  var goto_srv = new ROSLIB.Service({
    ros : ros,
    name : '/niryo_one/orthopus_interface/action',
    serviceType : 'niryo_one_msgs/SetInt'
  });
  goto_srv.callService(request, function(result) {
    console.log('Result for service call on '
      + goto_srv.name
      + ': '
      + result.status 
      + ', '
      + result.message);
  });
}

function GotoDrink() {
  console.log('GotoDrink');
  
  var request = new ROSLIB.ServiceRequest({
    value : ACTION_GOTO_DRINK
  });
  
  var goto_srv = new ROSLIB.Service({
    ros : ros,
    name : '/niryo_one/orthopus_interface/action',
    serviceType : 'niryo_one_msgs/SetInt'
  });
  goto_srv.callService(request, function(result) {
    console.log('Result for service call on '
    + goto_srv.name
    + ': '
    + result.status 
    + ', '
    + result.message);
  });
}

function FlipPinch() {
  console.log('FlipPinch');
  
  var request = new ROSLIB.ServiceRequest({
    value : ACTION_FLIP_PINCH
  });
  
  var goto_srv = new ROSLIB.Service({
    ros : ros,
    name : '/niryo_one/orthopus_interface/action',
    serviceType : 'niryo_one_msgs/SetInt'
  });
  goto_srv.callService(request, function(result) {
    console.log('Result for service call on '
    + goto_srv.name
    + ': '
    + result.status 
    + ', '
    + result.message);
  });
}

function Goto(positionName) {
//   console.log('Got to ' + positionName);
//   
//   SetJoystickEnable(false);
//   
//   joints_positions = position_map.get(positionName);
//     
//   var robotActionClient = new ROSLIB.ActionClient({
//     ros : ros,
//     serverName : '/niryo_one/commander/robot_action',
//     actionName : 'niryo_one_msgs/RobotMoveAction'
//   });
// 
//   var goal = new ROSLIB.Goal({
//     actionClient : robotActionClient,
//     goalMessage : {
//       cmd : {
//         cmd_type : CMD_JOINTS,
//         joints : joints_positions
//       }
//     }
//   });
// 
//   goal.on('feedback', function(feedback) {
//     console.log('Feedback: ' + feedback.state);
//   });
// 
//   goal.on('result', function(result) {
//     console.log('Final Result: ' + result.message);
//     GotoPoseEnd();
//   });
//   
//   goal.send();
}

function GotoPoseEnd() {

}

window.onload = function () {
  
var robot_IP = document.domain;
if(robot_IP=="")
{
  robot_IP="localhost"; 
}
var ros_master_uri="ws://"+robot_IP+":9090";

// Connecting to ROS
// -----------------
ros = new ROSLIB.Ros({
  url : ros_master_uri
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
}); 

// Setup topic subscribers
// ----------------------
learning_mode_sub = new ROSLIB.Topic({
  ros : ros,
  name : '/niryo_one/learning_mode',
  messageType : 'std_msgs/Bool'
});
learning_mode_sub.subscribe(function(message) {
  console.debug('Received message on ' + learning_mode_sub.name + ': ' + message.data);
  learning_mode = message.data;
  UpdatePage();
});

hardware_status_sub = new ROSLIB.Topic({
  ros : ros,
  name : '/niryo_one/hardware_status',
  messageType : 'niryo_one_msgs/HardwareStatus'
});
hardware_status_sub.subscribe(function(message) {
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
  ros : ros,
  name : '/niryo_one/joystick_interface/is_enabled',
  messageType : 'std_msgs/Bool'
});
joystick_enable_sub.subscribe(function(message) {
  console.debug('Received message on ' + joystick_enable_sub.name + ': ' + message.data);
  joystick_enabled = message.data;
  UpdatePage();
});

// Setup topic publishers 
// ----------------------
cmd_vel_pub = new ROSLIB.Topic({
      ros : ros,
      name : '/sim_joy',
      messageType : 'sensor_msgs/Joy'
    });

  // Construct joystick objects 
  // ----------------------
  var yz_joy = Joystick($('#YZ_joystick')[0], '#0066ff', Y_AXIS, true, Z_AXIS, false);
  var yx_joy = Joystick($('#YX_joystick')[0], '#0066ff', Y_AXIS, true, X_AXIS, false);
  var rot_yz_joy = Joystick($('#rotYZ_joystick')[0], '#0066ff', ROTY_AXIS, false, ROTZ_AXIS, false);
  var rot_yx_joy = Joystick($('#rotYX_joystick')[0], '#0066ff', ROTY_AXIS, false, ROTX_AXIS, false);

  // Populate video source 
  $('#video')[0].src = "http://" + robot_IP + ":8080/stream?topic=/image_raw";

  // Setup main loop timer
  // ----------------------
  mainTimer = setInterval(function(){mainLoop();}, 100);
}


$(function() {
    $('#enable_switch')
        .change(function() {
            var state =  $('#enable_switch')[0].checked;                    
            SetLearningMode(!state);
        });
    $('#cartesian_switch')
        .change(function() {
            var state =  $('#cartesian_switch')[0].checked;                    
            SetJoystickEnable(state);
        });
    $('#goto_home')
        .click(function() {
            GotoHome();
        });
    $('#goto_rest')
        .click(function() {
            GotoRest();
        });
    $('#goto_drink')
        .click(function() {
          GotoDrink();
        });
    $('#flip_pinch')
        .click(function() {
          FlipPinch();
        });
    $('#gripper_open')
        .click(function() {
            GripperOpen();
        });
    $('#gripper_close')
        .click(function() {
            GripperClose();
        });
});
