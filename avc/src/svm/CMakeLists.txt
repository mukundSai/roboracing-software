add_executable(svm svm.cpp)
target_link_libraries(color_svm_avc ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) 
