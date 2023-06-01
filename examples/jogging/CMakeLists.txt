cmake_minimum_required(VERSION 3.0.0)
project(jogging
    DESCRIPTION "Basic example to demonstrate sending joint velocity commands (jogging)."
    VERSION 0.1.0 # default, can be overridden by CI pipeline
    LANGUAGES CXX)

add_executable(${PROJECT_NAME}
    main.cpp
)

add_dependencies(${PROJECT_NAME}
    robcomm
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME}
    PUBLIC robcomm
    PRIVATE Threads::Threads
)
