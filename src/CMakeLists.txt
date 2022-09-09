set(CAN_SRCS
   CanSocket.cpp
   CanCommunication.cpp
   BMCCommunication.cpp
)

set(MAIN_SRCS
    main.cpp
)

add_library(CAN ${CAN_SRCS})

add_executable(MAIN ${MAIN_SRCS})
target_link_libraries(MAIN CAN)