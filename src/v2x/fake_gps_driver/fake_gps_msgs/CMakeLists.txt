cmake_minimum_required(VERSION 2.8.3)
project(fake_gps_msgs)


find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        rospy
        sensor_msgs
        std_msgs
        message_generation
)
add_message_files(
    FILES
        Ephemeris.msg
        L1L2Range.msg
        Inspvax.msg  #add by wendao
        GpsRange.msg
        Carrier.msg
        # L1Range.msg
        
        # DeltaPosition.msg
        
        # RpvData.msg
        # XYZRpvData.msg
        # ENURpvData.msg
        # Rpv.msg

        # Utm.msg   #add by wendao
        # Satellite.msg
        # Satellites.msg
        
        # Gpchc.msg
)
generate_messages(
    DEPENDENCIES
        sensor_msgs
        std_msgs
)


catkin_package(
    CATKIN_DEPENDS message_runtime sensor_msgs std_msgs
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)