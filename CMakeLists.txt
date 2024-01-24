cmake_minimum_required(VERSION 3.5)

project(AA-CCBS LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )

set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost COMPONENTS multi_index)
include_directories( "D:/boost_1_75_0" )
include_directories( "." )

include(GNUInstallDirs)

set( SOURCE_FILES
    main.cpp
    config.cpp
    tinyxml2.cpp
    xml_logger.cpp
    map.cpp
    task.cpp
    cbs.cpp
    to_aa_sipp.cpp)

set( HEADER_FILES
    const.h
    tinyxml2.h
    config.h
    xml_logger.h
    map.h
    task.h
    cbs.h
    structs.h
    lineofsight.h
    pheuristic.h
    states_container.h
    to_aa_sipp.h)

add_executable( AA-CCBS ${SOURCE_FILES} ${HEADER_FILES} )
install(TARGETS AA-CCBS
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
