cmake_minimum_required(VERSION 3.27)
project(feup_da_02)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
file(COPY data/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(feup_da_02
        main.cpp
        include/Graph.h
        include/MutablePriorityQueue.h
        include/Utilities.h             src/Utilities.cpp
        include/DataParser.h            src/DataParser.cpp
        include/TSP.h                   src/TSP.cpp
        include/Menu.h                  src/Menu.cpp
)

find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_INPUT ${PROJECT_SOURCE_DIR}/include)
    set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs)
    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_GENERATE_MAN NO)
    set(DOXYGEN_EXTRACT_PRIVATE YES)

    add_custom_target(
            docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_SOURCE_DIR}/docs/Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen" VERBATIM
    )

    add_dependencies(feup_da_02 docs)
endif(DOXYGEN_FOUND)
