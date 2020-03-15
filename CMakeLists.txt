project (cserial)

cmake_minimum_required(VERSION 3.5)


if(WIN32)
    message(STATUS "System: window")
    set(libcserial libcserial)
elseif(UNIX)
    message(STATUS "System: unix-like")
    set(libcserial cserial)
endif()
message(STATUS "install prefix: " ${CMAKE_INSTALL_PREFIX})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/build/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/build/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin)
add_executable(main 
    serial.c
    main.c
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/build/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/build/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/build/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/build/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/build/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/build/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/lib)
if(WIN32)
    add_library(${libcserial} STATIC
        serial.c
    )
elseif(UNIX)
    add_library(${libcserial} SHARED
        serial.c
    )
endif()


if(UNIX)
    install(DIRECTORY lib/ DESTINATION lib)
    install(DIRECTORY include/ DESTINATION include/de/io)
endif()