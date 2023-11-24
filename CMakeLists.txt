#=============================================================================
# CMake configuration file for Chrono-Projects
# 
#=============================================================================

cmake_minimum_required(VERSION 2.8)

project(SnakeProject)

# Set location of additional CMAKE modules
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Set location of executable
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)


#-----------------------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)
#-----------------------------------------------------------------------------

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif()

#-----------------------------------------------------------------------------
# Force C++11
#-----------------------------------------------------------------------------

#if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#    set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
#endif()

#-----------------------------------------------------------------------------
# Disable some warnings
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(MSVC)
        add_definitions( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-DNOMINMAX" ) # do not use MSVC's min/max macros
        add_definitions( "-MP" ) # perform parallel builds
    endif(MSVC)
endif()

#--------------------------------------------------------------
# Find the Chrono package with required components
#--------------------------------------------------------------

# Invoke find_package in CONFIG mode

find_package(Chrono
             OPTIONAL_COMPONENTS Irrlicht
             CONFIG
)

# If Chrono and/or the required component(s) were not found, return now.

if(NOT Chrono_FOUND)
  message("Could not find requirements for IRRLICHT projects")
  return()
endif()

#--------------------------------------------------------------
# Include paths and libraries
#--------------------------------------------------------------

# (A) Path to the Chrono include headers
# - If using an installed version of Chrono, this will be the path 
#   to the installed headers (the configuration headers are also
#   available there)
# - If using a build version of Chrono, this will contain both the
#   path to the Chrono sources and the path to the chrono BUILD tree
#   (the latter for the configuration headers)
# 
# (B) Path to the top of the source tree for this project
# - for access to utility headers

include_directories(
    ${CHRONO_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
	"include"
)

#--------------------------------------------------------------
# Compilation flags
#--------------------------------------------------------------

set(COMPILE_FLAGS ${CHRONO_CXX_FLAGS})

# Disable some warnings triggered by Irrlicht (Windows only)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(COMPILE_FLAGS "${COMPILE_FLAGS} /wd4275")
endif()

#--------------------------------------------------------------
# Loop over all demo programs and build them
#--------------------------------------------------------------

FILE(GLOB sources "src/*.cpp" "include/*.h")


add_executable(SnakeProject ${sources})
source_group(""  FILES ${sources})

set_target_properties(SnakeProject PROPERTIES
FOLDER demos
COMPILE_FLAGS "${COMPILE_FLAGS}"
COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
LINK_FLAGS "${COMPILE_FLAGS} ${CHRONO_LINKER_FLAGS}"
)

target_link_libraries(SnakeProject ${CHRONO_LIBRARIES})




#--------------------------------------------------------------
# Copy DLLs
#--------------------------------------------------------------

# Add custom command for copying DLLs to the appropriate binary
# output folder (on Windows only).

add_DLL_copy_command("${CHRONO_DLLS}")



