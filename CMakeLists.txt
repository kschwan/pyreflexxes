cmake_minimum_required(VERSION 3.1)
project(pyreflexxes)

find_package(pybind11 2.2 REQUIRED)
find_package(RMLTypeII)
find_package(RMLTypeIV)

if(NOT RMLTypeII_FOUND AND NOT RMLTypeIV_FOUND)
  message(FATAL_ERROR "Reflexxes Motion Libraries (either Type II or Type IV) not found")
endif()

if(RMLTypeII_FOUND)
  pybind11_add_module(rml_type_ii
    src/rmlerror.cpp
    src/positiontrajectory.cpp
    src/pyreflexxes.cpp
  )
  target_link_libraries(rml_type_ii PRIVATE RMLTypeII::ReflexxesTypeII)
  target_compile_definitions(rml_type_ii PRIVATE "RML_TYPE_II")
endif()

if(RMLTypeIV_FOUND)
  pybind11_add_module(rml_type_iv
    src/rmlerror.cpp
    src/positiontrajectory.cpp
    src/pyreflexxes.cpp
  )
  target_link_libraries(rml_type_iv PRIVATE RMLTypeIV::ReflexxesTypeIV)
  target_compile_definitions(rml_type_iv PRIVATE "RML_TYPE_IV")
endif()
