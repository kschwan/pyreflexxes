cmake_minimum_required(VERSION 3.1)
project(pyreflexxes)

find_package(pybind11 2.2 REQUIRED)
find_package(RMLTypeII)
find_package(RMLTypeIV)

if(NOT RMLTypeII_FOUND AND NOT RMLTypeIV_FOUND)
  message(FATAL_ERROR "Reflexxes Motion Libraries (either Type II or Type IV) not found")
endif()

if(RMLTypeII_FOUND)
  pybind11_add_module(reflexxes_type2 src/reflexxes_type2.cpp)
  target_link_libraries(reflexxes_type2 PRIVATE RMLTypeII::ReflexxesTypeII)
endif()

if(RMLTypeIV_FOUND)
  pybind11_add_module(reflexxes_type4 src/reflexxes_type4.cpp)
  target_link_libraries(reflexxes_type4 PRIVATE RMLTypeIV::ReflexxesTypeIV)
endif()
