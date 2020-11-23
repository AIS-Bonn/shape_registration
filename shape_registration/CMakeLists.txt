cmake_minimum_required(VERSION 2.8.3)
project(shape_registration)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  rqt_gui_cpp
  cpd
  sensor_msgs
)


# Decide on a Qt version
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    set(USE_QT5 true)
else()
    set(USE_QT5 false)
endif()


### Find QT
if(USE_QT5)  
    find_package(Qt5 COMPONENTS Core Gui REQUIRED)                                                                      
    find_package(Qt5Widgets REQUIRED)                                                                                   
    
    find_library(QWT_LIBRARY qwt-qt5)
    
    set(QT_LIBRARIES "${Qt5Widgets_LIBRARIES}" "${QT5Gui_LIBRARIES}") 
else()
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)                                                                  
    include(${QT_USE_FILE})                                                             
    
    find_library(QWT_LIBRARY qwt)
endif()

# Include directories for QWT
find_path(QWT_INCLUDE_DIR qwt.h HINTS /usr/include/qwt /usr/local/lib/qwt)
    
# VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# PCL
find_package(PCL COMPONENTS common visualization REQUIRED)

# Ceres
find_package(Ceres REQUIRED)

#Set compiler settings
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_AUTORCC ON) # Resource files
    
       
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES shape_registration
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}   # include this for ui_h
    ${QWT_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)

file(GLOB gui_ui "ui/*.ui")

if(USE_QT5)
    qt5_wrap_cpp(MOC_SRCS 
        include/shape_registration/shape_registration.hpp 
        include/shape_registration/solver_thread.hpp
        include/shape_registration/solver.hpp
    )
    
    qt5_wrap_cpp(MOC_SRCS_GUI  include/shape_registration/gui.hpp )
    qt5_wrap_ui(UI_HDRS_GUI ${gui_ui} )
else()
    qt4_wrap_cpp(MOC_SRCS 
        include/shape_registration/shape_registration.hpp
        include/shape_registration/solver_thread.hpp
        include/shape_registration/solver.hpp
    )
    
    qt4_wrap_cpp(MOC_SRCS_GUI  include/shape_registration/gui.hpp )
    qt4_wrap_ui(UI_HDRS_GUI ${gui_ui} )
endif()


add_library(learning_pca
    src/pca.cpp
)

target_link_libraries(learning_pca
    ${catkin_LIBRARIES}
)

add_library(shape_registration
    src/pclviewer.cpp 
    src/cpd.cpp 
    src/cloud_manager.cpp 
    src/shape_registration.cpp 
    src/shape_utils.cpp 
    src/mesh_manager.cpp
    src/solver_thread.cpp
    src/solver.cpp
    ${MOC_SRCS}
)

target_link_libraries(shape_registration
    learning_pca
    ${catkin_LIBRARIES}
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    ${QT_LIBRARIES}
    ${CERES_LIBRARIES}
)

add_executable(shape_registration_node 
    src/main.cpp 
)

target_link_libraries(shape_registration_node
    shape_registration
)

add_executable(shape_gui_node src/main_gui.cpp src/gui.cpp ${MOC_SRCS_GUI} ${UI_HDRS_GUI})

target_link_libraries(shape_gui_node
    shape_registration
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
    ${QWT_LIBRARY}
)
