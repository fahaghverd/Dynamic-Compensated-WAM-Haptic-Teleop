cmake_minimum_required(VERSION 2.6)
project(libbarrett_examples)


# By default, build programs with compiler optimizations and debugging symbols
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()


## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
message(STATUS "BARRETT_INCLUDE_DIRS: ${BARRETT_INCLUDE_DIRS}")
link_directories(${BARRETT_LIBRARY_DIRS})
message(STATUS "BARRETT_LIBRARY_DIRS: ${BARRETT_LIBRARY_DIRS}")
add_definitions(${BARRETT_DEFINITIONS})
message(STATUS "BARRETT_DEFINITIONS: ${BARRETT_DEFINITIONS}")

## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR} include)

function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} ${prog}.cpp)
		target_link_libraries(${prog} ${BARRETT_LIBRARIES})
	endforeach()
endfunction()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)  

add_programs(
	ex11_master_master_gravity_extorq
	ex11_master_master_gravity
	ex11_master_master_feed
	ex11_master_master_ctc_feed_extorq
	ex11_master_master_ID_feed_extorq
	ex11_master_master_nogravity
	ex11_master_master_feed_full
	ex11_master_master_feed_full_extorq

)

