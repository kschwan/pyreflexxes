cmake_minimum_required(VERSION 3.1)
project(pyreflexxes)

find_package(RMLTypeII REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
find_package(Boost REQUIRED python)

add_library(_reflexxes_type2 MODULE src/reflexxes_type2.cpp)
target_compile_options(_reflexxes_type2 PRIVATE -Wall)
set_target_properties(_reflexxes_type2 PROPERTIES PREFIX "")
target_include_directories(_reflexxes_type2 PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(_reflexxes_type2 RMLTypeII::ReflexxesTypeII Boost::python)
