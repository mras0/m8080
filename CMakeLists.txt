cmake_minimum_required(VERSION 3.7)
project(m8080 CXX)

if (MSVC)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Supported build configurations" FORCE)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest /permissive- /Zi /Zo /sdl /W4")
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4324") # C4324: Structure was padded due to alignment specifier
    if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /JMC")
    endif()
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /DEBUG")
    add_definitions("-D_CRT_SECURE_NO_WARNINGS")
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT invaders)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a -Wall -Wextra -pedantic -g")
endif()


if (WIN32)
    add_definitions("-DWIN32 -D_WIN32 -DUNICODE -D_UNICODE -D_WIN32_WINNT=0x600")
endif()

add_library(core
    ioutil.cpp
    ioutil.h
    m8080.cpp
    m8080.h
    )

add_executable(cpm_test
    cpm_test.cpp
    cpm.cpp
    cpm.h
    )
target_link_libraries(cpm_test PRIVATE core)

add_executable(invaders
    main.cpp
    display.cpp
    display.h
    )
target_link_libraries(invaders PRIVATE core)
