# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.3)
project(chimney_rviz_plugins)

set(TARGET_NAME ${PROJECT_NAME})

## Add support for C++11, supported in ROS Kinetic and newer
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

# Find the PCL library
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS rviz)

add_definitions(-DPKG_DIR="${CMAKE_CURRENT_LIST_DIR}")

catkin_package(
    DEPENDS rviz
    CATKIN_DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

# find all dir in include/
FILE(GLOB dirs
        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/*
)
FILE(GLOB HEADER_FILES
        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/*.h
)
if(HEADER_FILES)
    LIST(REMOVE_ITEM dirs ${HEADER_FILES})
endif()
# remove unsolved file
# set(ignored_include_dirs example)
if(ignored_include_dirs)
    LIST(REMOVE_ITEM dirs ${ignored_header_dirs})
endif()
FOREACH(target ${dirs})
    MESSAGE(STATUS "loading the dir: ${target}")
    # search for all h files in target dir
    FILE(GLOB_RECURSE ${target}_include
            RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
            *.h
    )
    LIST(APPEND HEADER_FILES ${${target}_include})
ENDFOREACH()

if(rviz_QT_VERSION VERSION_LESS "5")
    qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
else()
    qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
endif()

# find all dir in nodes/
FILE(GLOB dirs
        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        nodes/*
)
FILE(GLOB SRC_FILES
        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        nodes/*.c
        nodes/*.cpp
)
if(SRC_FILES)
    LIST(REMOVE_ITEM dirs ${SRC_FILES})
endif()
# remove unsolved file
# set(ignored_src_dirs example)
if(ignored_src_dirs)
    LIST(REMOVE_ITEM dirs ${ignored_src_dirs})
endif()
FOREACH(target ${dirs})
    MESSAGE(STATUS "loading the dir: ${target}")
    # search for all cpp files in target dir
    FILE(GLOB_RECURSE ${target}_srcs
            RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
            *.c
            *.cpp
    )
    LIST(APPEND SRC_FILES ${${target}_srcs})
ENDFOREACH()
LIST(APPEND SRC_FILES ${MOC_FILES})

add_library(${TARGET_NAME} ${SRC_FILES})

target_include_directories(${TARGET_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET_NAME} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

## Install rules

install(TARGETS
        ${TARGET_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
        plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
