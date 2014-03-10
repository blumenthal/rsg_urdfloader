#CATKIN HYDRO

cmake_minimum_required(VERSION 2.8.3)
project(rsg_urdfloader)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
)

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")
FIND_PACKAGE(BRICS_3D REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost COMPONENTS thread)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(OpenSceneGraph COMPONENTS osgGA osgUtil osgViewer osgText) 
FIND_PACKAGE(Xerces) 
ADD_DEFINITIONS(-DEIGEN3)


# add include directories (-I)
INCLUDE_DIRECTORIES(${BRICS_3D_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
        ${OPENSCENEGRAPH_INCLUDE_DIRS})

# add library directories (-L)
LINK_DIRECTORIES(${BRICS_3D_LINK_DIRECTORIES} )


catkin_package(
  #LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp  urdf
  DEPENDS orocos_kdl TinyXML
)

include_directories(include  ${TinyXML_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(rsg_urdfloader
#   src/${PROJECT_NAME}/rsg_urdfloader.cpp
# )

## Declare a cpp executable
add_executable(rsg_urdfloader_node src/rsg_urdfloader.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rsg_urdfloader_node 
        ${BRICS_3D_LIBRARIES}      
        ${XERCES_LIBRARIES}
        ${OSG_LIBRARY_DEBUG}
        ${OSGGA_LIBRARY_DEBUG}
        ${OSGVIEWER_LIBRARY_DEBUG}
        ${OSGUTIL_LIBRARY_DEBUG}
        ${OSGTEXT_LIBRARY_DEBUG}
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
 )


