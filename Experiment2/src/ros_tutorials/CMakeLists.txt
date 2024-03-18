cmake_minimum_required(VERSION 3.0.2)
project(ros_tutorials)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

#Generate messages in the 'msg' folder
add_message_files(
  FILES
  Counts.msg
)
add_message_files(
  FILES
  IsClap.msg
)

#Generate added messages and services with anu dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES ros_tutorials
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(number_generator_node src/node_1_number_generator.cpp)
add_executable(game_logic_node src/node_2_game_logic.cpp)
add_executable(game_output_node src/node_3_game_output.cpp)

target_link_libraries(number_generator_node
  ${catkin_LIBRARIES}
)

target_link_libraries(game_logic_node
  ${catkin_LIBRARIES}
)

target_link_libraries(game_output_node
  ${catkin_LIBRARIES}
)