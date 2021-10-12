aux_source_directory(. SRC_LIST)
# 将此目录的源文件集合称为变量SRC_LIST

message(${SRC_LIST})


include_directories(${PROJECT_SOURCE_DIR}/include)
# 头文件目录

add_library(VRP_shared SHARED ${SRC_LIST})
# 库的名称，类型，源文件（动态库）

# add_library(Solider_static STATIC ${SRC_LIST})
# 静态库

set_target_properties(VRP_shared PROPERTIES OUTPUT_NAME "VRP")
# 库的名称，库的输出名称（动态库）

# set_target_properties(Solider_static PROPERTIES OUTPUT_NAME "VRP")
# 静态库

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# 库文件输出到工作空间的lib目录下