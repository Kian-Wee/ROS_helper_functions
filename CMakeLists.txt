cmake_minimum_required(VERSION 2.8.3)
project(tf_publisher)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS std_msgs)
