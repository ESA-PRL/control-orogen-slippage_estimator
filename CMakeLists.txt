PROJECT(slippage_estimator)
cmake_minimum_required(VERSION 2.6)

find_package(Rock)
if (ROCK_USE_CXX11)
    rock_activate_cxx11()
endif()

SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
INCLUDE(slippage_estimatorBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

