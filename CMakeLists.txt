cmake_minimum_required(VERSION 3.5)

project(AA-CCBS LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O0 -Wall -Wextra" )
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
    heuristic.cpp
    sipp.cpp
    task.cpp
    cbs.cpp
    simplex/columnset.cpp
    simplex/constraint.cpp
    simplex/matrix.cpp
    simplex/objectivefunction.cpp
    simplex/simplex.cpp
    simplex/variable.cpp
    TO-AA-SIPP/to_aa_sipp.cpp)

set( HEADER_FILES
    const.h
    tinyxml2.h
    config.h
    xml_logger.h
    map.h
    heuristic.h
    sipp.h
    task.h
    cbs.h
    structs.h
    simplex/columnset.h
    simplex/constraint.h
    simplex/datamismatchexception.h
    simplex/divisionbyzeroexception.h
    simplex/indexoutofboundexception.h
    simplex/matrix.h
    simplex/matrixissingularexception.h
    simplex/matrixnotsquareexception.h
    simplex/memoryreachedoutexception.h
    simplex/notavectorexception.h
    simplex/objectivefunction.h
    simplex/pilal.h
    simplex/pilalexceptions.h
    simplex/simplex.h
    simplex/simplexexceptions.h
    simplex/sizemismatchexception.h
    simplex/variable.h
    TO-AA-SIPP/lineofsight.h
    TO-AA-SIPP/pheuristic.h
    TO-AA-SIPP/states_container.h
    TO-AA-SIPP/to_aa_sipp.h)

add_executable( AA-CCBS ${SOURCE_FILES} ${HEADER_FILES} )
install(TARGETS AA-CCBS
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
