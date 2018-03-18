set(SOURCES
    sls_detector_client.cpp
)

include_directories(
    ../commonFiles
    ../multiSlsDetector 
    ../slsDetector 
    ../slsDetectorAnalysis 
    ../slsReceiverInterface 
    ../usersFunctions 
    ../MySocketTCP
)

add_executable(sls_detector_get
    ${SOURCES}
)
target_link_libraries(sls_detector_get
   slsDetectorShared
   pthread
   zmq
   rt
)
set_target_properties(sls_detector_get PROPERTIES 
    COMPILE_DEFINITIONS GET=1
)

add_executable(sls_detector_put
    ${SOURCES}
)
target_link_libraries(sls_detector_put
   slsDetectorShared
   pthread
   zmq
   rt
)
set_target_properties(sls_detector_put PROPERTIES 
    COMPILE_DEFINITIONS PUT=1
)

add_executable(sls_detector_acquire
    ${SOURCES}
)
target_link_libraries(sls_detector_acquire
   slsDetectorShared
   pthread
   zmq
   rt
)
set_target_properties(sls_detector_acquire PROPERTIES 
    COMPILE_DEFINITIONS READOUT=1
)

add_executable(sls_detector_help
    ${SOURCES}
)
target_link_libraries(sls_detector_help
   slsDetectorShared
   pthread
   zmq
   rt
)
set_target_properties(sls_detector_help PROPERTIES 
    COMPILE_DEFINITIONS HELP=1
)

install(TARGETS sls_detector_get sls_detector_put sls_detector_acquire
		sls_detector_help
	DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
)
