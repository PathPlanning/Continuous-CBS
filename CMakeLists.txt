cmake_minimum_required( VERSION 2.8 )

project( CCBS )

set(Boost_USE_STATIC_LIBS OFF) 
find_package(Boost COMPONENTS multi_index)
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( "." )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )

set( SOURCE_FILES 
     main.cpp  
	 config.cpp
     tinyxml2.cpp
     xml_logger.cpp 
     map.cpp 
     heuristic.cpp 
     sipp.cpp 
     task.cpp
     cbs.cpp )

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
	 structs.h)

add_executable( CCBS ${SOURCE_FILES} ${HEADER_FILES} )
