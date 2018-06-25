set(SOURCES
    src/MySocketTCP.cpp
    src/UDPInterface.cpp
    src/UDPBaseImplementation.cpp
    src/UDPStandardImplementation.cpp
    src/slsReceiverTCPIPInterface.cpp
    src/slsReceiver.cpp
    src/slsReceiverUsers.cpp
    src/utilities.cpp
    src/File.cpp
    src/BinaryFile.cpp
    src/ThreadObject.cpp
    src/Listener.cpp
    src/DataProcessor.cpp
    src/DataStreamer.cpp
    src/Fifo.cpp
)


# HDF5
if (USE_HDF5)
	if (HDF5_FOUND)
	    include_directories( 
	        ${HDF5_INCLUDE_DIRS}
	    )
	    add_definitions( 
	        -DHDF5C ${HDF5_DEFINITIONS}
	    )
	    list (APPEND SOURCES 
	        src/HDF5File.cpp 
	        src/HDF5FileStatic.cpp
	    )
	endif ()
endif (USE_HDF5)


add_definitions(
    -DDACS_INT  -DSLS_RECEIVER_UDP_FUNCTIONS
)

include_directories(
    include
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
target_link_libraries(slsReceiverStatic
    numa
)
set_target_properties(slsReceiverStatic PROPERTIES
    ARCHIVE_OUTPUT_NAME SlsReceiver
)

add_library(slsReceiverShared SHARED
    ${SOURCES}
    ${HEADERS}
)
target_link_libraries(slsReceiverShared
    numa
)

set(PUBLICHEADERS
    include/sls_receiver_defs.h
    include/ansi.h
    include/sls_receiver_funcs.h
    include/MySocketTCP.h
    include/genericSocket.h

)


set_target_properties(slsReceiverShared PROPERTIES
    LIBRARY_OUTPUT_NAME SlsReceiver
    PUBLIC_HEADER "${PUBLICHEADERS}"
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

if (HDF5_FOUND)
    target_link_libraries(slsReceiver ${HDF5_LIBRARIES})
endif ()



install(TARGETS slsReceiverShared slsReceiverStatic slsReceiver
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include)

install(FILES ${ZMQ_STATIC_ARCHIVE}
	DESTINATION lib)
