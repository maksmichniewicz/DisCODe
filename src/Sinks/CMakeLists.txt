SET(CMAKE_INCLUDE_CURRENT_DIR ON)

MESSAGE(STATUS "Building sinks:")

ADD_SINK(CvWindow)
ADD_SINK(CvVideoWriter)
ADD_SINK(WAVSink)
ADD_SINK(Speakers)
