cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "../../../components/." "../../a76xx" "../../LedBicolor")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test-a76xx)
