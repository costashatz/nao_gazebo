cmake_minimum_required(VERSION 2.8.3)

#These are NAOqi SIM's known components (ie. libraries)
set(NAOqi_COMPONENTS
    alaudio
    albonjourdiscovery
    alextractor
    allog
    almodelutils
    alproject
    alresource
    altools
    alautomatictest
    alboxrary
    alfile
    almathinternal
    almotion
    alpythonbridge
    alserial
    altts
    albehaviorinfo
    alcommon
    allauncher
    almath
    almotionrecorder
    alpythontools
    alsoap
    alvalue
    albehavior
    alerror
    allogremote
    almemoryfastaccess
    alparammanager
    alremotecall
    althread
    alvision
    alproxies
    qi
    qitype
    alnaosim
    alsimutils
    alrobotmodel)


#Set INCLUDE hints
set(NAOqi_INCLUDE_HINTS
    "${NAOqi_SIM_DIR}/include"
    "$ENV{AL_SIM_DIR}/include" )

# Set LIBRARY hints
set(NAOqi_LIBRARY_HINTS
    "${NAOqi_SIM_DIR}/lib"
    "$ENV{AL_SIM_DIR}/lib" )

# Find include directories
find_path(NAOqiSIM_INCLUDE_DIR alcommon/alproxy.h HINTS ${NAOqi_INCLUDE_HINTS} )

# Verify we know about all the components requested
# and remove those we don't know about
set(NAOqi_FILTERED_COMPONENTS ${NAOqiSIM_FIND_COMPONENTS})

if ( NAOqiSIM_FIND_COMPONENTS )
    foreach(comp ${NAOqiSIM_FIND_COMPONENTS})
        list(FIND NAOqi_COMPONENTS ${comp} ${comp}_KNOWN)
        if (${comp}_KNOWN EQUAL -1)
            list(REMOVE_ITEM NAOqi_FILTERED_COMPONENTS ${comp})
            message(STATUS "Unknown NAOqi component ${comp}")
        endif()
    endforeach()
endif()

list(LENGTH NAOqi_FILTERED_COMPONENTS NAOqiSIM_NUMBER_OF_COMPONENTS)
set(NAOqiSIM_FOUND_COMPONENTS TRUE)

# Look for components (ie. libraries)
if( ${NAOqiSIM_NUMBER_OF_COMPONENTS}  )
    foreach(comp ${NAOqi_FILTERED_COMPONENTS})
        #Look for the actual library here
        find_library(${comp}_LIBRARY NAMES ${comp} HINTS ${NAOqi_LIBRARY_HINTS})
        if ( ${${comp}_LIBRARY} STREQUAL ${comp}_LIBRARY-NOTFOUND)
            message(STATUS "Could not find NAOqi's ${comp}")
            set(NAOqiSIM_FOUND_COMPONENTS FALSE)
        else()
            #If everything went well append this component to list of libraries
            list(APPEND NAOqiSIM_LIBRARY ${${comp}_LIBRARY})
        endif()
    endforeach()
else()
    message(STATUS "No NAOqi components specified")
endif()


# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    NAOqiSIM #Package name
    DEFAULT_MSG
    # Variables required to evaluate as TRUE
    NAOqiSIM_LIBRARY
    NAOqiSIM_INCLUDE_DIR
    NAOqiSIM_FOUND_COMPONENTS)

# Copy the values of the advanced variables to the user-facing ones
set(NAOqiSIM_LIBRARIES ${NAOqiSIM_LIBRARY} )
set(NAOqiSIM_INCLUDE_DIRS ${NAOqiSIM_INCLUDE_DIR} )
set(NAOqiSIM_FOUND ${NAOQISIM_FOUND})

# If NAOqi was found, update NAOqi_DIR to show where it was found
if ( NAOqiSIM_FOUND )
  get_filename_component(NAOqi_NEW_DIR "${NAOqiSIM_INCLUDE_DIRS}/../" ABSOLUTE)
endif()
set(NAOqiSIM_DIR ${NAOqi_NEW_DIR} CACHE FILEPATH "NAOqi root directory" FORCE)

#Hide these variables
mark_as_advanced(NAOqiSIM_INCLUDE_DIR NAOqiSIM_LIBRARY NAOQISIM_FOUND)
