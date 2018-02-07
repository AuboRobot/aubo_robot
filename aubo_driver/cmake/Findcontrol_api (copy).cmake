# libcontrolAPI_INCLUDE_DIRS - the libcontrolAPI include directories
# libcontrolAPI_LIBS - link these to use libcontrolAPI


find_path(libcontrolAPI_INCLUDE_DIR
	NAMES our_control_api.h
	PATHS ${PROJECT_SOURCE_DIR}/include/aubo_driver/control_api
	      $ENV{INCLUDE}
)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	find_library(libcontrolAPI
		NAMES controlAPI
		PATHS ${PROJECT_SOURCE_DIR}/lib/lib64
	)

	find_library(libev
		NAMES ev
		PATHS ${PROJECT_SOURCE_DIR}/lib/lib64
	)
else()
        find_library(libcontrolAPI
                NAMES controlAPI
                PATHS ${PROJECT_SOURCE_DIR}/lib/lib32
        )

        find_library(libev
                NAMES ev
                PATHS ${PROJECT_SOURCE_DIR}/lib/lib32
        )
endif()


set(libcontrolAPI_INCLUDE_DIRS ${libcontrolAPI_INCLUDE_DIR})

set(libcontrolAPI_LIBS ${libcontrolAPI})


if(libcontrolAPI_INCLUDE_DIRS)
	message(STATUS "Found Control API include dir: ${libcontrolAPI_INCLUDE_DIRS}")
else(libcontrolAPI_INCLUDE_DIRS)
	message(STATUS "Could NOT find Control API headers.")
endif(libcontrolAPI_INCLUDE_DIRS)


if(libcontrolAPI_LIBS)
	message(STATUS "Found Control API library: ${libcontrolAPI_LIBS}")
else(libcontrolAPI_LIBS)
	message(STATUS "Could NOT find libcontrolAPI library.")
endif(libcontrolAPI_LIBS)

if(libcontrolAPI_INCLUDE_DIRS AND libcontrolAPI_LIBS)
	set(libcontrolAPI_FOUND TRUE)
else(libcontrolAPI_INCLUDE_DIRS AND libcontrolAPI_LIBS)
	set(libcontrolAPI_FOUND FALSE)
	if(libcontrolAPI_FIND_REQUIRED)
		message(FATAL_ERROR "Could not find Control API.")
	endif(libcontrolAPI_FIND_REQUIRED)
endif(libcontrolAPI_INCLUDE_DIRS AND libcontrolAPI_LIBS)
