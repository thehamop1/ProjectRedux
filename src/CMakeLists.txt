set(CAN_SRCS
   Socket.cpp
)

set(MAIN_SRCS
    main.cpp
)

add_library(CAN ${CAN_SRCS})
target_include_directories(CAN PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(MAIN ${MAIN_SRCS})
target_link_libraries(MAIN CAN)