cmake_minimum_required(VERSION 4.1)
project(stacja_pogodowa C)

set(CMAKE_C_STANDARD 11)

add_library(stacja_pogodowa STATIC library.c)
