cmake_minimum_required(VERSION 3.10)
project(dqrobotics_wasm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Emscripten 编译标志
if(EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX ".js")
    
    # 添加 DQ Robotics 源目录
    set(DQROBOTICS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics-cpp)
    
    # 查找 Eigen3
    # 注意: 在 Emscripten 环境中，你需要确保 Eigen 可用
    # 可以通过设置 EIGEN3_INCLUDE_DIR 或使用 emscripten 的 ports
    find_package(Eigen3 QUIET)
    
    if(NOT EIGEN3_FOUND)
        # 如果找不到系统的 Eigen，使用本地副本
        set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../eigen)
        message(STATUS "Using local Eigen at: ${EIGEN3_INCLUDE_DIR}")
    endif()
    
    # 添加 DQ Robotics 库的源文件
    set(DQROBOTICS_SOURCES
        ${DQROBOTICS_DIR}/src/DQ.cpp
        ${DQROBOTICS_DIR}/src/internal/_dq_linesegment.cpp
        ${DQROBOTICS_DIR}/src/utils/DQ_Geometry.cpp
        ${DQROBOTICS_DIR}/src/utils/DQ_LinearAlgebra.cpp
        ${DQROBOTICS_DIR}/src/utils/DQ_Math.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_Kinematics.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialManipulator.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialManipulatorDH.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_SerialManipulatorMDH.cpp
        ${DQROBOTICS_DIR}/src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
        ${DQROBOTICS_DIR}/src/robots/FrankaEmikaPandaRobot.cpp
        ${DQROBOTICS_DIR}/src/robots/KukaLw4Robot.cpp
    )
    
    # 创建 WASM 模块
    add_executable(dqrobotics
        bindings.cpp
        ${DQROBOTICS_SOURCES}
    )
    
    # 包含目录
    target_include_directories(dqrobotics PRIVATE
        ${DQROBOTICS_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
    )
    
    # Emscripten 链接标志
    set(EMSCRIPTEN_LINK_FLAGS
        "-lembind"
        "-s WASM=1"
        "-s ALLOW_MEMORY_GROWTH=1"
        "-s MODULARIZE=1"
        "-s EXPORT_NAME='createDQRoboticsModule'"
        "-s ENVIRONMENT='web,worker'"
        "-s SINGLE_FILE=1"
        "-s DISABLE_EXCEPTION_CATCHING=0"
        "-s ASSERTIONS=1"
        "--bind"
        "-O3"
    )
    
    # 将标志转换为字符串
    string(REPLACE ";" " " EMSCRIPTEN_LINK_FLAGS_STR "${EMSCRIPTEN_LINK_FLAGS}")
    
    # 设置链接器标志
    set_target_properties(dqrobotics PROPERTIES
        LINK_FLAGS "${EMSCRIPTEN_LINK_FLAGS_STR}"
    )
    
    # 安装规则
    install(TARGETS dqrobotics
        RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../public/wasm
    )
    
    message(STATUS "Configured DQ Robotics WASM build")
    message(STATUS "DQ Robotics directory: ${DQROBOTICS_DIR}")
    message(STATUS "Eigen3 include: ${EIGEN3_INCLUDE_DIR}")
    
else()
    message(FATAL_ERROR "This project must be built with Emscripten")
endif()

