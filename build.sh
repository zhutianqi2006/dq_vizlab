#!/bin/bash
# DQ Robotics WASM 构建脚本

set -e

echo "================================"
echo "DQ Robotics WASM 构建脚本"
echo "================================"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查 Emscripten 是否安装
if ! command -v emcc &> /dev/null; then
    echo -e "${RED}错误: Emscripten 未安装${NC}"
    echo "请访问 https://emscripten.org/docs/getting_started/downloads.html 安装 Emscripten"
    echo ""
    echo "快速安装命令:"
    echo "  git clone https://github.com/emscripten-core/emsdk.git"
    echo "  cd emsdk"
    echo "  ./emsdk install latest"
    echo "  ./emsdk activate latest"
    echo "  source ./emsdk_env.sh"
    exit 1
fi

echo -e "${GREEN}✓ Emscripten 已安装${NC}"
emcc --version | head -n 1

# 检查是否需要下载 Eigen
EIGEN_DIR="./eigen"
if [ ! -d "$EIGEN_DIR" ]; then
    echo -e "${YELLOW}正在下载 Eigen 库...${NC}"
    git clone --depth 1 --branch 3.4.0 https://gitlab.com/libeigen/eigen.git $EIGEN_DIR
    echo -e "${GREEN}✓ Eigen 下载完成${NC}"
else
    echo -e "${GREEN}✓ Eigen 已存在${NC}"
fi

# 创建构建目录
BUILD_DIR="./build"
if [ -d "$BUILD_DIR" ]; then
    echo -e "${YELLOW}清理旧的构建目录...${NC}"
    rm -rf $BUILD_DIR
fi

mkdir -p $BUILD_DIR
mkdir -p ./public/wasm

echo -e "${YELLOW}配置 CMake...${NC}"
cd $BUILD_DIR

# 使用 Emscripten 工具链配置 CMake
emcmake cmake ../wasm \
    -DEIGEN3_INCLUDE_DIR=../eigen \
    -DCMAKE_BUILD_TYPE=Release

echo -e "${YELLOW}开始编译...${NC}"
emmake make -j$(nproc)

echo -e "${YELLOW}安装到 public/wasm...${NC}"
emmake make install

cd ..

echo ""
echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}构建成功!${NC}"
echo -e "${GREEN}================================${NC}"
echo ""
echo "WASM 模块已生成在: ./public/wasm/"
echo ""
echo "下一步:"
echo "  1. 运行 'npm install' 安装前端依赖"
echo "  2. 运行 'npm run dev' 启动开发服务器"
echo "  3. 在浏览器中打开 http://localhost:5173"
echo ""

