#
# Copyright 2014-2015 Ettus Research LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

cmake_minimum_required(VERSION 2.8)

### Set up build environment ##################################################
# Choose a static or shared-library build (shared is default, and static will
# probably need some special care!)
# Set this to ON in order to link a static build of UHD:
option(UHD_USE_STATIC_LIBS OFF)

SET(FFTW_INC_SEARCHPATH

 /usr/include

 /usr/include/fftw

 /usr/local/include

 /usr/local/include/fftw

 )

 SET(FFTW_LIB_SEARCHPATH

 /usr/lib

 /usr/lib/fftw

 /usr/local/lib

 /usr/local/lib/fftw

 )
 FIND_PATH(FFTW_INCLUDE_PATH fftw3.h ${FFTW_INC_SEARCHPATH})

 IF(FFTW_INCLUDE_PATH)

 SET(FFTW_INCLUDE ${FFTW_INCLUDE_PATH})

 ENDIF (FFTW_INCLUDE_PATH)

 FIND_LIBRARY(FFTW_LIB_PATH fftw3 ${FFTW_LIB_SEARCHPATH})

 IF(FFTW_LIB_PATH)
 SET(FFTW_LIB ${FFTW_LIB_PATH})

 ENDIF (FFTW_LIB_PATH)

 IF(FFTW_INCLUDE)

 IF(FFTW_LIB)

 SET(FFTW_FOUND 1)
 ENDIF (FFTW_LIB)
 ENDIF(FFTW_INCLUDE)

 IF(FFTW_FOUND)
message(STATUS "Found FFTW")
 INCLUDE_DIRECTORIES( ${FFTW_INCLUDE})
 LINK_LIBRARIES( ${FFTW_LIB} ) # modification

 ADD_DEFINITIONS(-DUSE_FFTW)

 SET(FFTW_DIR .)
 ENDIF (FFTW_FOUND)
 
MARK_AS_ADVANCED(FFTW_INCLUDE_PATH FFTW_LIB_PATH)
#############################################################################

# This example also requires Boost:
set(BOOST_REQUIRED_COMPONENTS
    program_options
    system
    thread
)
if(MSVC)
    set(BOOST_ALL_DYN_LINK "${BOOST_ALL_DYN_LINK}" CACHE BOOL "boost enable dynamic linking")
    if(BOOST_ALL_DYN_LINK)
        add_definitions(-DBOOST_ALL_DYN_LINK) #setup boost auto-linking in msvc
    else(BOOST_ALL_DYN_LINK)
        set(BOOST_REQUIRED_COMPONENTS) #empty components list for static link
    endif(BOOST_ALL_DYN_LINK)
endif(MSVC)
find_package(Boost "1.46" REQUIRED ${BOOST_REQUIRED_COMPONENTS})

# To add UHD as a dependency to this project, add a line such as this:
find_package(UHD "3.8.0" REQUIRED)
# The version in  ^^^^^  here is a minimum version.
# To specify an exact version:
#find_package(UHD 3.8.1 EXACT REQUIRED)

###############################GPS#############################################
INCLUDE(FindPackageHandleStandardArgs)
#INCLUDE(HandleLibraryTypes)

SET(GPS_IncludeSearchPaths
  /usr/include/
  /opt/local/include/
)

SET(GPS_LibrarySearchPaths
  /usr/lib/
  /opt/local/lib/
)

FIND_PATH(GPS_INCLUDE_DIR
  NAMES gps.h
  PATHS ${GPS_IncludeSearchPaths}
)
FIND_LIBRARY(GPS_LIBRARY_OPTIMIZED
  NAMES gps
  PATHS ${GPS_LibrarySearchPaths}
)

 INCLUDE_DIRECTORIES( ${GPS_INCLUDE_DIR})
 LINK_LIBRARIES( ${GPS_LIBRARY_OPTIMIZED} ) # modification
 message("Included GPS Libraries")

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
# The package is found if all variables listed are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GPS "Could NOT find gps/gpsd library. Install using sudo apt-get install gpsd libgps-dev. Read Ubuntu installation instructions at http://pixhawk.ethz.ch/software/installation/ubuntu"
  GPS_LIBRARY_OPTIMIZED
  GPS_INCLUDE_DIR
)


# Collect optimized and debug libraries
#HANDLE_LIBRARY_TYPES(GPS)

MARK_AS_ADVANCED(
  GPS_INCLUDE_DIR
  GPS_LIBRARY_OPTIMIZED
)


################################################################################
### Configure Compiler ########################################################
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
)

link_directories(${Boost_LIBRARY_DIRS})

### Make the executable #######################################################
set(SOURCES_MASDR
    masdr.cpp
    masdr.h
    utils.cpp
    utils.h
    #lsm303dlhc_mag.c
    #lsm303dlhc.h
    ## Add any other MASDR files here ##
)
add_executable(masdr ${SOURCES_MASDR})

SET(CMAKE_BUILD_TYPE "Release")

# Shared library case: All we need to do is link against the library, and
# anything else we need (in this case, some Boost libraries):
if(NOT UHD_USE_STATIC_LIBS)
    message(STATUS "Linking against shared UHD library.")
    target_link_libraries(masdr ${UHD_LIBRARIES} ${Boost_LIBRARIES})
# Shared library case: All we need to do is link against the library, and
# anything else we need (in this case, some Boost libraries):
else(NOT UHD_USE_STATIC_LIBS)
    message(STATUS "Linking against static UHD library.")
    target_link_libraries(masdr
        # We could use ${UHD_LIBRARIES}, but linking requires some extra flags,
        # so we use this convenience variable provided to us
        ${UHD_STATIC_LIB_LINK_FLAG}
        # Also, when linking statically, we need to pull in all the deps for
        # UHD as well, because the dependencies don't get resolved automatically
        ${UHD_STATIC_LIB_DEPS}
    )
endif(NOT UHD_USE_STATIC_LIBS)

### Doxygen command ###########################################################
add_custom_target(doc doxygen ${CMAKE_SOURCE_DIR}/config.dox)

### Once it's built... ########################################################
# Here, you would have commands to install your program.
# We will skip these in this example.
