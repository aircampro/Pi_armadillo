cmake_minimum_required(VERSION 3.10)
if(APPLE)
set(CMAKE_OSX_DEPLOYMENT_TARGET "10.14" CACHE STRING "Minimum OS X deployment version")
endif(APPLE)
project(RemoteCli LANGUAGES CXX)

### Append project cmake script dir ###
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


set(CMAKE_INCLUDE_CURRENT_DIR ON)

### Enumerate project files ###
include(enum_cli_hdr)
include(enum_cli_src)
include(enum_crsdk_hdr)

### Define output target ###
set(remotecli "${PROJECT_NAME}")
add_executable(${remotecli}
    ${cli_hdrs}
    ${cli_srcs}
    ${crsdk_hdrs}
)

if(APPLE)
    set_target_properties(${remotecli} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
        BUILD_RPATH "@executable_path"
    )
endif(APPLE)

if(NOT APPLE)
    set_target_properties(${remotecli} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
        BUILD_RPATH "$ORIGIN"
        INSTALL_RPATH "$ORIGIN"
    )
endif(NOT APPLE)

## Specify char is signed-char to fix mismatch with Raspbian
target_compile_options(${remotecli}
    PRIVATE
        -fsigned-char
)


set(MAV_LIB_DIRS /home/pi/cams/CrSDK_v1.05.00_20211207a_Linux32ARMv7/app/c_library_v2 )
set(LBOOST_FILE_SYS /usr/lib/arm-linux-gnueabihf)
SET (CMAKE_C_FLAGS “-Wall -std=c++17 -fvisibility=hidden -fvisibility-inlines-hidden”)
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREADED ON)
SET(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.74.0 COMPONENTS date_time coroutine filesystem graph log_setup program_options random regex serialization system thread timer log REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG true)
find_package(Threads REQUIRED)
# Find Poco package libraries 
find_package(Poco REQUIRED COMPONENTS Foundation Util Net XML JSON)
#find_package(CURL)
include_directories( ${Boost_INCLUDE_DIR} )
# add this for mavlink
#include_directories(" ${CMAKE_CURRENT_SOURCE_DIR}/c_library_v2, ${CMAKE_CURRENT_SOURCE_DIR}/c_library_v2/common", "${CMAKE_CURRENT_SOURCE_DIR}//c_library_v2/ardupilotmega")

## Add the poco libraries
if(MSVC) # WIN32
    SET(POCO_INCLUDE_DIRS "C:/Program Files/Poco/include")
else()
    SET(POCO_INCLUDE_DIRS "/usr/local/include/Poco")
endif(MSVC)
set(POCO_LIB_DIR "/usr/local/lib")
set(POCO_LIBS "${POCO_LIB_DIR}/libPocoFoundation.so"
             "${POCO_LIB_DIR}/libPocoXML.so"
             "${POCO_LIB_DIR}/libPocoJSON.so"
             "${POCO_LIB_DIR}/libPocoNet.so"
             "${POCO_LIB_DIR}/libPocoData.so"
             "${POCO_LIB_DIR}/libPocoUtil.so")


find_library(PTHREAD_LIBRARY pthread)

link_directories( ${Boost_LIBRARY_DIRS} )

target_include_directories(${remotecli}
    PRIVATE
        ${crsdk_hdr_dir} # defined in enum script
        ${MAV_LIB_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${LBOOST_FILE_SYS}
        ${POCO_INCLUDE_DIRS}
)
message("Include dirs of Boost : " ${Boost_INCLUDE_DIR} )
MESSAGE( [Main] " Poco_INCLUDE_DIRS = ${POCO_INCLUDE_DIRS}")
MESSAGE( [Main] " Poco_LIBRARIES = ${Poco_LIBRARIES}")
MESSAGE( [Main] " Poco_LIBS = ${Poco_LIBS}")


### Configure external library directories ###
set(ldir ${CMAKE_CURRENT_SOURCE_DIR}/external)
set(cr_ldir ${ldir}/crsdk)

### Link CRSDK library
find_library(camera_remote Cr_Core HINTS ${cr_ldir} )
find_library(loglib libboost_log.a )
#find_library(wiringPI_LIB wiringPi )
find_library(BOOST_LOG_LIBRARY boost_log)
find_library(BOOST_LOG_SETUP_LIBRARY boost_log_setup)
find_library(BOOST_THREAD_LIBRARY boost_thread)
find_library(BOOST_SYSTEM_LIBRARY boost_system)
find_library(BOOST_FILESYSTEM_LIBRARY boost_filesystem)

target_link_libraries(${remotecli}
    PRIVATE
        ${camera_remote}
        ${Boost_LIBRARIES}
        ${Boost_Log_LIBRARIES}
        ${PTHREAD_LIBRARY}
#        Threads::Threads
        ${loglib}
        ${Boost_PROGRAM_OPTIONS_LIBRARY} 
        ${Boost_REGEX_LIBRARY}
        ${BOOST_THREAD_LIBRARY}
        ${BOOST_SYSTEM_LIBRARY}
        ${BOOST_FILESYSTEM_LIBRARY}
        ${BOOST_LOG_SETUP_LIBRARY}
        ${BOOST_LOG_LIBRARY}
#        ${wiringPI_LIB}
#        Boost::filesystem
        ${CMAKE_THREAD_LIBS_INIT}
#        -lpthread
        ${POCO_LIBRARIES}
        ${POCO_LIBS}
)
#set(CMAKE_EXE_LINKER_FLAGS "-Ofast -lboost_log -lboost_system -lboost_regex -Lboost_thread -lboost_log_setup -lboost_timer -lstdc++fs -lexplain" CACHE STRING "GCC LD_FLAGS" FORCE)
#set(CMAKE_EXE_LINKER_FLAGS "-lpthread -lboost_log -lboost_system -lboost_regex -lboost_thread -lboost_log_setup -lboost_timer -lstdc++fs -lexplain" CACHE STRING "GCC LD_FLAGS" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lboost_log -lboost_system -lboost_regex -lboost_thread -lpthread -lboost_log_setup -lboost_timer -lstdc++fs -lexplain -DBOOST_DYN_LINK -DBOOST_LOG_DYN_LINK -DBOOST_BIND_GLOBAL_PLACEHOLDERS" CACHE STRING "GCC LD_FLAGS" FORCE)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lwiringPi -pthread -lboost_log -lboost_system -lboost_regex -lboost_thread -lpthread -lboost_log_setup -lboost_timer -lstdc++fs -lexplain -DBOOST_DYN_LINK -DBOOST_LOG_DYN_LINK -DBOOST_BIND_GLOBAL_PLACEHOLDERS" CACHE STRING "GCC LD_FLAGS" FORCE)
add_definitions( "-DBOOST_ALL_DYN_LINK" )
#add_definitions( -DBOOST_LOG_DYN_LINK )
add_definitions( "-Wall -std=c++17" )
add_definitions( "-DBOOST_LOG_DYN_LINK" )
add_definitions( "-DBOOST_BIND_GLOBAL_PLACEHOLDERS" )

#set(CMAKE_EXE_LINKER_FLAGS "-pthread -lboost_log -lboost_system -lboost_regex -lboost_thread -lboost_log_setup -lboost_timer -lstdc++fs -lexplain" CACHE STRING "GCC LD_FLAGS" FORCE)
MESSAGE("Camera Libraries = " ${camera_remote})
MESSAGE("Boost Libraries = " ${Boost_LIBRARIES})
### Windows specific configuration ###
if(WIN32)
    ## Build with unicode on Windows
    target_compile_definitions(${remotecli} PRIVATE UNICODE _UNICODE)
endif(WIN32)

### Linux specific configuration ###
if(UNIX AND NOT APPLE)
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8)
            # Must use std::experimental namespace if older than GCC8
            message("[${PROJECT_NAME}] GCC version less than 8. Using std::experimental namespace.")
            target_compile_definitions(${remotecli} PRIVATE USE_EXPERIMENTAL_FS)
        endif()

        if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
            # Must explicitly link separate std::filesystem if older than GCC9
            message("[${PROJECT_NAME}] GCC version less than 9. Explicitly linking separate std::filesystem library.")
            target_link_libraries(${remotecli} PRIVATE stdc++fs)
        endif()
    endif()
endif(UNIX AND NOT APPLE)

## Copy required library binaries
if(WIN32)
    add_custom_command(TARGET ${remotecli} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${cr_ldir} $<TARGET_FILE_DIR:${remotecli}>
    )
endif(WIN32)

if(UNIX AND NOT APPLE)
    add_custom_command(TARGET ${remotecli} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${cr_ldir} $<TARGET_FILE_DIR:${remotecli}>
    )
endif(UNIX AND NOT APPLE)

if(APPLE)
    set(ct_fw "Contents/Frameworks")
    set(adaptor_dir_name "CrAdapter")
    add_custom_command(TARGET ${remotecli} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${remotecli}>/${ct_fw}/${adaptor_dir_name}
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${cr_ldir} $<TARGET_FILE_DIR:${remotecli}>
        COMMAND ${CMAKE_COMMAND} -E copy_directory $<TARGET_FILE_DIR:${remotecli}>/${adaptor_dir_name} $<TARGET_FILE_DIR:${remotecli}>/${ct_fw}/${adaptor_dir_name}
        COMMAND ${CMAKE_COMMAND} -E rm -rf $<TARGET_FILE_DIR:${remotecli}>/${adaptor_dir_name}
    )
endif(APPLE)

#
# this is how you sepcify specific target compilations
#
#target_compile_options(${remotecli}
#  PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>
#  PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
#  PRIVATE $<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:Debug>>:-ggdb3>
#)

#set(CMAKE_CXX_FLAGS “-std=c++17” CACHE STRING “G++ CXX_FLAGS” FORCE)
#set(CMAKE_CXX_FLAGS “${CMAKE_CXX_FLAGS} -std=c++17”)
#set(CMAKE_EXE_LINKER_FLAGS “-lboost_log -lpthread -DBOOST_LOG_DYN_LINK -lboost_system -lboost_regex -lboost_thread -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer -lstdc++fs -DBOOST_BIND_GLOBAL_PLACEHOLDERS” CACHE STRING “GCC LD_FLAGS” FORCE)
#set(CMAKE_EXE_LINKER_FLAGS “${CMAKE_EXE_LINKER_FLAGS} -lboost_log -lpthread -DBOOST_LOG_DYN_LINK -lboost_system -lboost_regex -lboost_thread -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer -lstdc++fs -DBOOST_BIND_GLOBAL_PLACEHOLDERS”)

## Install application
## '.' means, install to the root directory of CMAKE_INSTALL_PREFIX
install(TARGETS ${remotecli} DESTINATION .)
install(DIRECTORY ${cr_ldir}/ DESTINATION .)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__ARMV8")
set(CMAKE_EXE_LINKER_FLAGS "-pthread -lboost_log -lboost_system -lboost_regex -lboost_thread -lboost_log_setup -lboost_timer -lstdc++fs -lexplain -lboost_filesystem" CACHE STRING "GCC LD_FLAGS" FORCE)
