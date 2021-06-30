set(AuBo_DIR ${PREBUILT_DIR}/aubo_dependents)

# 定义头文件路径
set(AuBo_INCLUDE_DIRS
        ${AuBo_DIR}/libconfig/linux_x64/inc;
        ${AuBo_DIR}/log4cplus/linux_x64/inc;
        ${AuBo_DIR}/protobuf/linux_x64/google/protobuf/inc;
        ${AuBo_DIR}/robotController/Header;
        ${AuBo_DIR}/robotSDK/inc;
        ${AuBo_DIR}/otg/inc)

# 链接库的路径
set(AuBo_LINK_DIRS
        ${AuBo_DIR}/protobuf/linux-x64/lib
        ${AuBo_DIR}/libconfig/linux_x64/lib
        ${AuBo_DIR}/log4cplus/linux_x64/lib
        ${AuBo_DIR}/robotController/linux_x64
        ${AuBo_DIR}/robotSDK/lib/linux_x64
        ${AuBo_DIR}/otg/lib)

SET(AuBo_LIBRARIES config;log4cplus;protobuf;protobuf-lite;protoc;auborobotcontroller;otgLib)

#FOREACH(aubo_component ${AuBo_LIB_COMPONENTS})
#    find_library(lib_${aubocomponent} NAMES ${aubocomponent} PATHS
#            ${AuBo_DIR}/protobuf/linux-x64/lib
#            ${AuBo_DIR}/libconfig/linux_x64/lib
#            ${AuBo_DIR}/log4cplus/linux_x64/lib
#            ${AuBo_DIR}/robotController/linux_x64
#            ${AuBo_DIR}/robotSDK/lib/linux_x64
#            NO_DEFAULT_PATH)
#    set(AuBo_LIBRARIES ${AuBo_LIBRARIES};${aubo_component})
#ENDFOREACH()
#MESSAGE(STATUS "AuBo_LIBRARIES: " ${AuBo_LIBRARIES})