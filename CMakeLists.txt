cmake_minimum_required( VERSION 2.8 )

project( CCBS )

set(Boost_USE_STATIC_LIBS OFF) 
find_package(Boost COMPONENTS multi_index)
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( "include" )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )

set( SOURCE_FILES 
     src/main.cpp  
	src/config.cpp
     src/collision.cpp
     src/tinyxml2.cpp
     src/xml_logger.cpp 
     src/json_logger.cpp
     src/map.cpp 
     src/heuristic.cpp 
     src/sipp.cpp 
     src/task.cpp
     src/cbs.cpp 
	src/simplex/columnset.cpp
	src/simplex/constraint.cpp
	src/simplex/matrix.cpp
	src/simplex/objectivefunction.cpp
	src/simplex/simplex.cpp
	src/simplex/variable.cpp)

set( HEADER_FILES 
     include/const.h  
     include/tinyxml2.h
	include/config.h
     include/collision.h
     include/xml_logger.h 
     include/json_logger.h
     include/map.h 
     include/heuristic.h 
     include/sipp.h 
     include/task.h
     include/cbs.h
	include/structs.h
	include/simplex/columnset.h
	include/simplex/constraint.h
	include/simplex/datamismatchexception.h
     include/simplex/divisionbyzeroexception.h
	include/simplex/indexoutofboundexception.h
	include/simplex/matrix.h
	include/simplex/matrixissingularexception.h
	include/simplex/matrixnotsquareexception.h
     include/simplex/memoryreachedoutexception.h
     include/simplex/notavectorexception.h
     include/simplex/objectivefunction.h
     include/simplex/pilal.h
     include/simplex/pilalexceptions.h
	include/simplex/simplex.h
	include/simplex/simplexexceptions.h
	include/simplex/sizemismatchexception.h
	include/simplex/variable.h)

add_executable( CCBS ${SOURCE_FILES} ${HEADER_FILES} )
