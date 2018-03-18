set(SOURCES
    src/MySocketTCP.cpp
    src/UDPInterface.cpp
    src/UDPBaseImplementation.cpp
    src/UDPStandardImplementation.cpp
    src/slsReceiverTCPIPInterface.cpp
    src/slsReceiver.cpp
    src/slsReceiverUsers.cpp
    src/utilities.cpp
)

add_definitions(
    -DDACS_INT  -DSLS_RECEIVER_UDP_FUNCTIONS
)

include_directories(
    include
    ../slsDetectorCalibration
)

add_library(zmq STATIC IMPORTED GLOBAL)

set(ZMQ_STATIC_ARCHIVE ${CMAKE_CURRENT_SOURCE_DIR}/include/libzmq.a)
set_target_properties(zmq PROPERTIES
    IMPORTED_LOCATION ${ZMQ_STATIC_ARCHIVE}
)

add_library(slsReceiverStatic STATIC
    ${SOURCES}
    ${HEADERS}
)
set_target_properties(slsReceiverStatic PROPERTIES
    ARCHIVE_OUTPUT_NAME SlsReceiver
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

add_library(slsReceiverShared SHARED
    ${SOURCES}
    ${HEADERS}
)
set_target_properties(slsReceiverShared PROPERTIES
    LIBRARY_OUTPUT_NAME SlsReceiver
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

add_executable(slsReceiver
    src/main.cpp
)
target_link_libraries(slsReceiver
    slsReceiverShared
    pthread
    zmq
    rt
)

install(TARGETS slsReceiver
	DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
)
install(TARGETS slsReceiverStatic slsReceiverShared
	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
)
install(FILES ${ZMQ_STATIC_ARCHIVE}
	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
)
