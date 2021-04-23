# Install script for directory: /home/xhe/offboard_ws/src/mavros-master/mavros_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/xhe/offboard_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/msg" TYPE FILE FILES
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/ADSBVehicle.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/ActuatorControl.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/Altitude.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/AttitudeTarget.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/BatteryStatus.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/CamIMUStamp.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/CommandCode.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/CompanionProcessStatus.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/OnboardComputerStatus.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/DebugValue.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/ESCInfo.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/ESCInfoItem.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/ESCStatus.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/ESCStatusItem.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/EstimatorStatus.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/ExtendedState.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/FileEntry.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/GlobalPositionTarget.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/GPSRAW.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/GPSRTK.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/HilActuatorControls.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/HilControls.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/HilGPS.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/HilSensor.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/HilStateQuaternion.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/HomePosition.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/LandingTarget.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/LogData.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/LogEntry.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/ManualControl.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/Mavlink.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/MountControl.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/OpticalFlowRad.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/OverrideRCIn.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/Param.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/ParamValue.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/PlayTuneV2.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/PositionTarget.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/RCIn.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/RCOut.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/RTCM.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/RadioStatus.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/RTKBaseline.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/State.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/StatusText.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/Thrust.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/TimesyncStatus.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/Trajectory.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/VFR_HUD.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/VehicleInfo.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/Vibration.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/Waypoint.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/WaypointList.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/WaypointReached.msg"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/msg/WheelOdomStamped.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/srv" TYPE FILE FILES
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/CommandBool.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/CommandHome.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/CommandInt.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/CommandLong.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/CommandTOL.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/CommandTriggerControl.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/CommandTriggerInterval.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/CommandVtolTransition.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/FileChecksum.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/FileClose.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/FileList.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/FileMakeDir.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/FileOpen.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/FileRead.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/FileRemove.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/FileRemoveDir.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/FileRename.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/FileTruncate.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/FileWrite.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/LogRequestData.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/LogRequestEnd.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/LogRequestList.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/MountConfigure.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/MessageInterval.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/ParamGet.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/ParamPull.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/ParamPush.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/ParamSet.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/SetMavFrame.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/SetMode.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/StreamRate.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/VehicleInfoGet.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/WaypointClear.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/WaypointPull.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/WaypointPush.srv"
    "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/srv/WaypointSetCurrent.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES "/home/xhe/offboard_ws/build/mavros-master/mavros_msgs/catkin_generated/installspace/mavros_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/xhe/offboard_ws/devel/include/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/xhe/offboard_ws/devel/share/roseus/ros/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/xhe/offboard_ws/devel/share/common-lisp/ros/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/xhe/offboard_ws/devel/share/gennodejs/ros/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/xhe/offboard_ws/devel/lib/python2.7/dist-packages/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/xhe/offboard_ws/devel/lib/python2.7/dist-packages/mavros_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/xhe/offboard_ws/build/mavros-master/mavros_msgs/catkin_generated/installspace/mavros_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES "/home/xhe/offboard_ws/build/mavros-master/mavros_msgs/catkin_generated/installspace/mavros_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES
    "/home/xhe/offboard_ws/build/mavros-master/mavros_msgs/catkin_generated/installspace/mavros_msgsConfig.cmake"
    "/home/xhe/offboard_ws/build/mavros-master/mavros_msgs/catkin_generated/installspace/mavros_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs" TYPE FILE FILES "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mavros_msgs" TYPE DIRECTORY FILES "/home/xhe/offboard_ws/src/mavros-master/mavros_msgs/include/mavros_msgs/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

