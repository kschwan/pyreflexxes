cmake_minimum_required(VERSION 3.1)
project(pyreflexxes)

# Use C++14
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)

find_package(PythonLibs 2.7 REQUIRED)
find_package(Boost REQUIRED python)

find_package(RMLTypeII)
find_package(RMLTypeIV)

if(NOT RMLTypeII_FOUND AND NOT RMLTypeIV_FOUND)
  message(FATAL_ERROR "Reflexxes Motion Libraries (either Type II or Type IV) not found")
endif()

if(RMLTypeII_FOUND)
add_library(reflexxes_type2 MODULE src/reflexxes_type2.cpp)
target_compile_options(reflexxes_type2 PRIVATE -Wall)
set_target_properties(reflexxes_type2 PROPERTIES PREFIX "")
target_include_directories(reflexxes_type2 PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(reflexxes_type2 RMLTypeII::ReflexxesTypeII Boost::python)
endif()

if(RMLTypeIV_FOUND)
add_library(reflexxes_type4 MODULE src/reflexxes_type4.cpp)
target_compile_options(reflexxes_type4 PRIVATE -Wall)
set_target_properties(reflexxes_type4 PROPERTIES PREFIX "")
target_include_directories(reflexxes_type4 PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(reflexxes_type4 RMLTypeIV::ReflexxesTypeIV Boost::python)
endif()
