add_executable(basic_camera_display basic_camera_display.cpp)

target_link_libraries(basic_camera_display
    ${realsense2_LIBRARY}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(basic_measure basic_measure.cpp)
target_link_libraries(basic_measure
                      ${realsense2_LIBRARY}
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES} 
                      Threads::Threads)
