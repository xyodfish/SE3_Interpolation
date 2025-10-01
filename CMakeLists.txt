cmake_minimum_required(VERSION 3.16)

# 项目名称和版本号，必填，版本号用于版本管理
project(SE3_Interpolation VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "The target C++ standard.")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 打印助手
include(CMakePrintHelpers)
set(CMAKE_BUILD_TYPE Release) # Debug

if ( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    add_compile_options(-g)
endif()


# 多平台部署
if (NOT ARCH_FREFIX)
    set(ARCH_FREFIX ${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}-${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION})
endif ()

# 默认使用Release构建类型
if ( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
    add_compile_options(-g)
    message( STATUS "Setting build type to '${CMAKE_BUILD_TYPE}'" )
else ( NOT CMAKE_BUILD_TYPE )
    message( STATUS "Build type set to '${CMAKE_BUILD_TYPE}' by user." )
endif ( NOT CMAKE_BUILD_TYPE )

# 设定默认安装路径
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/../../devel/${ARCH_FREFIX} CACHE PATH "Default installation directory" FORCE)
endif()

# 设置三方库开发平台路径
if (NOT ROOT_DEVEL_ARCH_PATH)
    set(ROOT_DEVEL_ARCH_PATH /opt/galbot/devel/${ARCH_FREFIX})
endif ()
cmake_print_variables(CMAKE_INSTALL_PREFIX)

# 设定默认三方库配置文件查找路径
list(APPEND CMAKE_PREFIX_PATH 0 ${CMAKE_INSTALL_PREFIX})
list(APPEND CMAKE_PREFIX_PATH ${ROOT_DEVEL_ARCH_PATH})
cmake_print_variables(CMAKE_PREFIX_PATH)
# 打印第三方库配置文件寻找过程
set(CMAKE_FIND_DEBUG_MODE 0)


# 查找依赖
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(THIRD_PARTY_INC ${PROJECT_SOURCE_DIR}/3rd/include)
set(THIRD_PARTY_LIB ${PROJECT_SOURCE_DIR}/3rd/lib)



# 下面是自定义的构建
include_directories(
    include
    ${INC_DIR}
)

add_subdirectory(src)


