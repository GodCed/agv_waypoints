##############################################################################
# CMake configuration
##############################################################################


cmake_minimum_required(VERSION 2.8.3)
project(agv_waypoints)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ui)


##############################################################################
# Dependencies and include paths
##############################################################################


find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    geometry_msgs
    move_base_msgs
)

find_package(OpenCV REQUIRED
    core
    imgproc
)

find_package(Qt5 COMPONENTS
    Widgets
    REQUIRED
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


##############################################################################
# Catkin package
##############################################################################

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        roscpp
        actionlib
        geometry_msgs
        move_base_msgs
)


##############################################################################
# GUI
##############################################################################


# UI files
set(QT_FORMS
    ui/main_window.ui
)

# Ressources
set(QT_RESOURCES
    resources/images.qrc
)

# Headers of QObject classes (for MOC)
set(QT_HEADERS
    include/agv_waypoints/main_window.hpp
    include/agv_waypoints/qnode.hpp
)

# Sources
set(QT_SOURCES
    src/main.cpp
    src/main_window.cpp
    src/qnode.cpp
    src/waypoint_list_item.cpp
    src/waypoint.cpp
)

# Target
add_executable(run_planner ${QT_HEADERS} ${QT_SOURCES} ${QT_RESOURCES} ${QT_FORMS})
target_link_libraries(run_planner Qt5::Widgets ${catkin_LIBRARIES})


##############################################################################
# Install
##############################################################################


# Binaries
install(
    TARGETS run_planner
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

