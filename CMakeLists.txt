# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.3)
project(chimney_rviz_plugins)

set(TARGET_NAME ${PROJECT_NAME})

## Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

include(auto_detect.cmake)

# Find the PCL library
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS rviz)

add_definitions(-DPKG_DIR="${CMAKE_CURRENT_LIST_DIR}")

catkin_package(
    DEPENDS rviz
    CATKIN_DEPENDS
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

## set the ignored nodes here
# set(ignored_include_dirs example)
# set(ignored_src_dirs example)

## If the advanced_plugins is exist, add them into build
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/advanced_plugins)
    MESSAGE(STATUS "advanced plugins found")
    CHIMNEY_INCLUDE_ALL(advanced_plugins/include)
    CHIMNEY_SOURCE_ALL(advanced_plugins/nodes)
    set(ADVANCED_PLUGINS_INCLUDE_DIR advanced_plugins/include)
endif()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${ADVANCED_PLUGINS_INCLUDE_DIR}
)

## search for all .h files in include dir
CHIMNEY_INCLUDE_ALL(include)
## search for all .c && .cpp files in include dir
CHIMNEY_SOURCE_ALL(nodes)

## add QT moc files into source list
if(rviz_QT_VERSION VERSION_LESS "5")
    qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})
else()
    qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})
endif()
LIST(APPEND SRC_FILES ${MOC_FILES})

add_library(${TARGET_NAME} ${SRC_FILES})

target_include_directories(${TARGET_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET_NAME} ${QT_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

## Install rules

install(TARGETS ${TARGET_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
        plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
