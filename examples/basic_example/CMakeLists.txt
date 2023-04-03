cmake_minimum_required(VERSION 3.0.0)
project(basic_example
    DESCRIPTION "Basic example to demonstrate the usage of librobcomm."
    VERSION 0.0.0 # default, can be overridden by CI pipeline
    LANGUAGES CXX)

add_executable(${PROJECT_NAME}
    main.cpp
)

add_dependencies(${PROJECT_NAME}
    robcomm
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC robcomm
)
