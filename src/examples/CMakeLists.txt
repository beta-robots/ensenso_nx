# Build example 1
ADD_EXECUTABLE(ex1_simple_capture ex1_simple_capture.cpp)
TARGET_LINK_LIBRARIES(ex1_simple_capture ${PROJECT_NAME}_LIB)
