# Build example 1
ADD_EXECUTABLE(ex1_simple_capture ex1_simple_capture.cpp)
TARGET_LINK_LIBRARIES(ex1_simple_capture ${PROJECT_NAME}_LIB)

# Build example 2
ADD_EXECUTABLE(ex2_save_depth_image ex2_save_depth_image.cpp)
TARGET_LINK_LIBRARIES(ex2_save_depth_image ${PROJECT_NAME}_LIB)
