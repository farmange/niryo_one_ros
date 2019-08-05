cmake_minimum_required(VERSION 2.8.8)

# Stable 3.0 seems to be the only working one
include(ExternalProject)
ExternalProject_Add(qpoases_svn
  PREFIX qpoases_svn
  SVN_REPOSITORY https://projects.coin-or.org/svn/qpOASES/stable/3.0
  SVN_TRUST_CERT 1
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

project(qpoases_ros)

find_package(catkin REQUIRED)

catkin_package(
  #INCLUDE_DIRS TODO
  LIBRARIES qpOASES
)

ExternalProject_Get_Property(qpoases_svn SOURCE_DIR)

# include_directories(${SOURCE_DIR}/include)

# "We do not recommend using GLOB to collect a list of source files from your source tree."
set(qpOASES_SRC
  ${SOURCE_DIR}/src/BLASReplacement.cpp
  ${SOURCE_DIR}/src/Bounds.cpp
  ${SOURCE_DIR}/src/Constraints.cpp
  ${SOURCE_DIR}/src/Flipper.cpp
  ${SOURCE_DIR}/src/Indexlist.cpp
  ${SOURCE_DIR}/src/LAPACKReplacement.cpp
  ${SOURCE_DIR}/src/Matrices.cpp
  ${SOURCE_DIR}/src/MessageHandling.cpp
  ${SOURCE_DIR}/src/Options.cpp
  ${SOURCE_DIR}/src/OQPinterface.cpp
  ${SOURCE_DIR}/src/QProblemB.cpp
  ${SOURCE_DIR}/src/QProblem.cpp
  ${SOURCE_DIR}/src/SolutionAnalysis.cpp
  ${SOURCE_DIR}/src/SQProblem.cpp
  ${SOURCE_DIR}/src/SubjectTo.cpp
  ${SOURCE_DIR}/src/Utils.cpp
)

set(qpOASES_PYTHON_SRC
  ${SOURCE_DIR}/interfaces/python/qpoases.pxd
  ${SOURCE_DIR}/interfaces/python/qpoases.pyx
)

# the files listed above are present when qpoases_svn (i.e. getting source files) is completed
add_custom_command(OUTPUT ${qpOASES_SRC} ${qpOASES_PYTHON_SRC}
  DEPENDS qpoases_svn)

# Custom build Python wrappers
add_custom_target(qpOASES_Python ALL
  COMMAND python setup.py build_ext --include-dirs ${SOURCE_DIR}/include ${INCLUDE_DIRECTORIES} --build-lib ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION} --build-temp ${CMAKE_CURRENT_BINARY_DIR} --sources ${qpOASES_PYTHON_SRC} ${qpOASES_SRC}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DEPENDS qpoases_svn
  COMMENT "Building Python wrappers."
)

# Tell CMake about the build output
set(qpOASES_PYTHON_LIB
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/qpoases.so
)

add_custom_command(OUTPUT ${qpOASES_PYTHON_LIB}
  DEPENDS qpOASES_Python)

# Install
install(FILES ${qpOASES_PYTHON_LIB}
  DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
)
