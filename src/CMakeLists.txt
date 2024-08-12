cmake_minimum_required(VERSION 3.10)
project(learn_rbdl)

find_package(RBDL REQUIRED URDFREADER REQUIRED)
find_package(Eigen3 REQUIRED)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(${PROJECT_NAME}
                main.cpp
)
# target_include_directories(${PROJECT_NAME}
#   PUBLIC
  
# )
target_link_libraries(${PROJECT_NAME}
                      ${RBDL_LIBRARY}
                      ${RBDL_URDFREADER_LIBRARY}
)
