find_path(libneoclient_INCLUDE_DIR
	NAMES robot.h
	PATHS ${PROJECT_SOURCE_DIR}/include/neo_driver/api
	      $ENV{INCLUDE}
)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	find_library(libneoclient
		NAMES NeoClient
		PATHS ${PROJECT_SOURCE_DIR}/lib/lib64
	)

else()
	find_library(libneoclient
		NAMES NeoClient
		PATHS ${PROJECT_SOURCE_DIR}/lib/lib32
	)

endif()


set(libneoclient_INCLUDE_DIRS ${libneoclient_INCLUDE_DIR})

set(libneoclient_LIBS ${libneoclient})

if(libneoclient_INCLUDE_DIRS)
	message(STATUS "Found API include dir: ${libneocobot_INCLUDE_DIRS}")
else()
	message(STATUS "Could NOT find API headers.")
endif()

if(libneoclient_LIBS)
	message(STATUS "Found API library: ${libneocobot_LIBS}")
else()
	message(STATUS "Could NOT find libneocobot library.")
endif()

if(libneoclient_INCLUDE_DIRS AND libneoclient_LIBS)
	set(libneoclient_FOUND TRUE)
else()
	set(libneoclient_FOUND FALSE)
endif()