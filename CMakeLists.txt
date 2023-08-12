# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
if($ENV{IDF_PATH})
    set(IDF_PATH $ENV{IDF_PATH})
else()
    set(IDF_PATH /Users/alistair/esp/ESP8266_RTOS_SDK)
    set(ENV{IDF_PATH} ${IDF_PATH})
endif()
set(PYTHON /Users/alistair/esp/venv/bin/python)
set(PROJECT_NAME "epsolar")
set(PROJECT_VER "0.0.6")
include(${IDF_PATH}/tools/cmake/project.cmake)
project(epsolar)
