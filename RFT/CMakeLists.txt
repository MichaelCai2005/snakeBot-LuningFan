 
cmake_minimum_required (VERSION 2.8)
project ("snake")

# change the vartiables to your path
INCLUDE_DIRECTORIES (./include)
INCLUDE_DIRECTORIES (D:/Library/irrlicht-1.8.1/include)
INCLUDE_DIRECTORIES (D:/Library/ChronoEngine/include/bullet)

set(chronodir D:/Library/ChronoEngine)
INCLUDE_DIRECTORIES (${chronodir}/include/chrono)
LINK_DIRECTORIES (${chronodir}/lib)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	ADD_DEFINITIONS(-DNOMINMAX)
endif()


#find_package(GLEW REQUIRED)
#find_package(GLUT REQUIRED)
#add_subdirectory (controller)

FILE(GLOB sources "src/*.cpp")
FILE(GLOB headers "include/*.h")
add_executable(main ${headers} ${sources})
find_library(chronolib NAMES ChronoEngine PATHS ${chronodir}/lib)
find_library(chronoirrlichtlib NAMES ChronoEngine_IRRLICHT PATHS ${chronodir}/lib)
find_library(irrlichtlib NAMES Irrlicht)

target_link_libraries(main ${chronolib} ${chronoirrlichtlib} ${irrlichtlib})