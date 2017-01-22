#SET(COMPILE_FLAGS "-mcmodel=large")

#SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${COMPILE_FLAGS}" )

#set(CMAKE_CXX_FLAGS                " ${CMAKE_CXX_FLAGS_INIT} -std=c++11 -fPIC -mcmodel=large")
#set(CMAKE_CXX_FLAGS_DEBUG          "-g -DDEBUG")
#set(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
#set(CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG")
#set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DDEBUG")

#set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

add_llvm_loadable_module(LLVMRegAllocCB 
    RegAllocCB.cpp
    )
