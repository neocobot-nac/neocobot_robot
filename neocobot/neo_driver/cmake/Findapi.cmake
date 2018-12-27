find_path(libneocobot_INCLUDE_DIR
	NAMES Robot.h
	PATHS ${PROJECT_SOURCE_DIR}/include/neo_driver/api
	      $ENV{INCLUDE}
)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	find_library(libneocobot
		NAMES neocobot
		PATHS ${PROJECT_SOURCE_DIR}/lib/lib64
	)

	find_library(libtinyxml
		NAMES tinyxml
		PATHS ${PROJECT_SOURCE_DIR}/lib/lib64
	)
else()
	find_library(libneocobot
		NAMES neocobot
		PATHS ${PROJECT_SOURCE_DIR}/lib/lib32
	)

	find_library(libtinyxml
		NAMES tinyxml
		PATHS ${PROJECT_SOURCE_DIR}/lib/lib32
	)
endif()


set(libneocobot_INCLUDE_DIRS ${libneocobot_INCLUDE_DIR})

set(libneocobot_LIBS ${libneocobot})

set(libtinyxml_LIBS ${libtinyxml})

if(libneocobot_INCLUDE_DIRS)
	message(STATUS "Found API include dir: ${libneocobot_INCLUDE_DIRS}")
else(libneocobot_INCLUDE_DIRS)
	message(STATUS "Could NOT find API headers.")
endif(libneocobot_INCLUDE_DIRS)


if(libneocobot_LIBS)
	message(STATUS "Found API library: ${libneocobot_LIBS}")
else(libneocobot_LIBS)
	message(STATUS "Could NOT find libneocobot library.")
endif(libneocobot_LIBS)

if(libneocobot_INCLUDE_DIRS AND libneocobot_LIBS)
	set(libneocobot_FOUND TRUE)
else(libneocobot_INCLUDE_DIRS AND libneocobot_LIBS)
	set(libneocobot_FOUND FALSE)
	if(libneocobot_FIND_REQUIRED)
		message(FATAL_ERROR "Could not find API.")
	endif(libneocobot_FIND_REQUIRED)
endif(libneocobot_INCLUDE_DIRS AND libneocobot_LIBS)