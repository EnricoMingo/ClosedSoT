# Copyright: (C) 2021 MultiDof
# Authors: Enrico Mingo Hoffman
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.0)
project(closed_sot VERSION 1.0.0)

add_compile_options(-std=c++11)

option(COMPILE_TESTS "Compile tests" FALSE)

find_package(cartesian_interface)
find_package(catkin REQUIRED COMPONENTS cartesian_interface
                                        roscpp
                                        urdf
                                        rospy
                                        std_msgs
                                        message_generation)
find_package(OpenSoT REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(matlogger2 REQUIRED)



###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

    INCLUDE_DIRS

    LIBRARIES
    ClosedSoT

    
    )

include_directories(include
    ${OpenSoT_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})
    
add_library(ClosedSoT SHARED
    src/tasks/velocity/ClosedChain.cpp)

target_link_libraries(ClosedSoT
    ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${VISP_LIBRARIES})

install(TARGETS ClosedSoT
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

add_library(CartesioClosedSoT SHARED
                cartesio/tasks/velocity/ClosedChain.cpp)

target_link_libraries(CartesioClosedSoT ClosedSoT ${cartesian_interface_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS CartesioClosedSoT
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )



install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h")



if(COMPILE_TESTS)
#  enable_testing()
#    add_subdirectory(tests)
endif()



