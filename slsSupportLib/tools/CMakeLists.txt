add_executable(sls_geometry_assembler geometry_assembler.cpp)
target_link_libraries(sls_geometry_assembler 
    slsSupportStatic
    pthread
    rt
)
set_target_properties(sls_geometry_assembler PROPERTIES 
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

install(TARGETS sls_geometry_assembler DESTINATION bin)
