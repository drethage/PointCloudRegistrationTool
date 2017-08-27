cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-dev -Ofast")
project(Point_Cloud_Registration_Tool)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(HEADER_DIR ${PROJECT_SOURCE_DIR}/include)
set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
set(SOURCE_FILES ${SOURCE_DIR}/main.cpp ${SOURCE_DIR}/util.cpp ${SOURCE_DIR}/Visualizer.cpp ${SOURCE_DIR}/Registrator.cpp) 
set(HEADER_FILES ${HEADER_DIR}/types.hpp ${HEADER_DIR}/util.hpp ${HEADER_DIR}/Visualizer.hpp ${HEADER_DIR}/Registrator.hpp) 
option(WITH_OPENMP "WITH_OPENMP" ON)

add_subdirectory(${PROJECT_SOURCE_DIR}/extern/gflags)
find_package(PCL 1.7 REQUIRED COMPONENTS common io kdtree features visualization)

if(WITH_OPENMP)
find_package(OpenMP)

if(OPENMP_FOUND)
	message("OpenMP found")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

endif()

if(PCL_FOUND)
	message("PCL found")

	include_directories(${PCL_INCLUDE_DIRS})
	include_directories(${HEADER_DIR})
	
	add_definitions(${PCL_DEFINITIONS})

	add_executable(point_cloud_registration_tool ${SOURCE_FILES} ${HEADER_FILES})

    target_link_libraries(point_cloud_registration_tool gflags)
    target_link_libraries(point_cloud_registration_tool ${PCL_COMMON_LIBRARIES} 
	${PCL_IO_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
endif()
