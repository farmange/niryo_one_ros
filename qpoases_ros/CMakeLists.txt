cmake_minimum_required(VERSION 2.8.8)

project(qpoases_ros)


find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS 3.2/include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(3.2/include)

set(qpOASES_SRC
  3.2/src/BLASReplacement.cpp
  3.2/src/Bounds.cpp
  3.2/src/Constraints.cpp
  3.2/src/Flipper.cpp
  3.2/src/Indexlist.cpp
  3.2/src/LAPACKReplacement.cpp
  3.2/src/Matrices.cpp
  3.2/src/MessageHandling.cpp
  3.2/src/Options.cpp
  3.2/src/OQPinterface.cpp
  3.2/src/QProblem.cpp
  3.2/src/QProblemB.cpp
  3.2/src/SolutionAnalysis.cpp
  3.2/src/SparseSolver.cpp
  3.2/src/SQProblem.cpp
  3.2/src/SQProblemSchur.cpp
  3.2/src/SubjectTo.cpp
  3.2/src/Utils.cpp
)

add_library(${PROJECT_NAME} ${qpOASES_SRC})
