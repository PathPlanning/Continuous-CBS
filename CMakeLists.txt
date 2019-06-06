cmake_minimum_required( VERSION 2.8 )

project( C-CBS )

include_directories( "." )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )

set( SOURCE_FILES 
     main.cpp  
     tinyxml2.cpp
     xml_logger.cpp 
     graph.cpp 
     heuristic.cpp 
     sipp.cpp 
     task.cpp
     cbs.cpp )

set( HEADER_FILES 
     const.h  
     tinyxml2.h
     xml_logger.h 
     graph.h 
     heuristic.h 
     sipp.h 
     task.h
     cbs.h
	 structs.h)

add_executable( C-CBS ${SOURCE_FILES} ${HEADER_FILES} )
