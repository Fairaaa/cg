cmake_minimum_required(VERSION 2.8)
project(stb_image C)

add_library(stb_image STATIC stb_image.c)
target_include_directories(stb_image PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})