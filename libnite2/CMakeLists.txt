cmake_minimum_required(VERSION 2.8.3)
project(libnite2)

find_package(catkin REQUIRED)

catkin_destinations() # set-up destination variables
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include(ExternalProject)
ExternalProject_Add(EP_${PROJECT_NAME}
    URL https://github.com/ipa320/thirdparty/raw/master/NiTE-Linux-x64-2.2.tar.bz2
    URL_MD5 5160387adbfb19830cfb61d26ba7d56d

    SOURCE_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src
    BUILD_COMMAND ""
        # copy headers to devel space (catkin does not like headers in source space)
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src/include/libnite2 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    INSTALL_COMMAND ""
)

catkin_package(
    CFG_EXTRAS libnite2-extras.cmake # catkin_package does not support artifacts in devel space
    EXPORTED_TARGETS EP_${PROJECT_NAME}
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
