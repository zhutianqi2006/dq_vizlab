# DQ Robotics 可视化实验室

基于 WebAssembly 的机器人运动学可视化平台，使用对偶四元数（Dual Quaternion）进行机器人运动学计算和可视化。

## 🌟 主要特性

### 核心功能
- **单臂机器人控制**：支持 Franka Panda、KUKA LBR iiwa 等机器人模型
- **双臂机器人控制**：支持双机械臂协同操作，包括绝对位姿和相对位姿控制
- **实时 3D 可视化**：基于 Three.js 的高性能 3D 渲染
- **逆运动学求解**：使用伪逆雅可比方法进行实时 IK 计算
- **URDF 模型支持**：支持加载标准 URDF 机器人模型文件
- **MDH 参数提取**：从 URDF 自动提取并显示 MDH 参数

### 交互功能
- **关节角度控制**：滑块和数值输入框，支持实时调整
- **示教器点动控制**：6 自由度末端执行器控制（平移 + 旋转）
- **坐标系统切换**：局部坐标系和世界坐标系切换
- **角度单位切换**：支持度（°）和弧度（rad）显示
- **关节角度导入/导出**：一键复制和导入关节角度配置
- **关节限位管理**：自动读取并应用关节限位约束
- **预设位姿**：快速切换到常用机器人姿态

### 技术亮点
- **WebAssembly 加速**：核心计算在 WASM 中执行，性能优异
- **DQ Robotics C++**：使用对偶四元数进行精确的运动学计算
- **实时同步**：关节角度、3D 模型、末端位姿实时同步更新

## 📦 技术栈

- **前端框架**：Vite + Vanilla JavaScript
- **3D 渲染**：Three.js
- **运动学库**：DQ Robotics C++ (编译为 WebAssembly)
- **数学库**：Eigen (C++)
- **模型加载**：URDF Loader
- **配置解析**：js-yaml

## 🚀 快速开始

### 前置要求

1. **Node.js** (推荐 v18+)
2. **Emscripten** (用于编译 WASM 模块)
   ```bash
   git clone https://github.com/emscripten-core/emsdk.git
   cd emsdk
   ./emsdk install latest
   ./emsdk activate latest
   source ./emsdk_env.sh
   ```

### 安装步骤

1. **克隆仓库**
   ```bash
   git clone <repository-url>
   cd dq_vizlab
   ```

2. **初始化子模块**（Eigen 和 DQ Robotics C++）
   ```bash
   git submodule update --init --recursive
   ```

3. **构建 WASM 模块**
   ```bash
   ./build.sh
   ```
   或使用 npm 脚本：
   ```bash
   npm run build:wasm
   ```

4. **安装前端依赖**
   ```bash
   npm install
   ```

5. **启动开发服务器**
   ```bash
   npm run dev
   ```

6. **访问应用**
   打开浏览器访问 `http://localhost:5173`

### 构建生产版本

```bash
npm run build
```

构建产物将输出到 `dist/` 目录。

## 📖 使用指南

### 单臂机器人控制

1. **选择机器人类型**：在控制面板中选择机器人（如 Franka Panda）
2. **调整关节角度**：
   - 使用滑块快速调整
   - 或直接在输入框中输入精确角度值
3. **使用示教器点动**：
   - 点击点动按钮（+X, -X, +Y, -Y, +Z, -Z 等）
   - 长按按钮持续移动
   - 切换"局部增量 / 世界坐标"模式改变坐标系

### 双臂机器人控制

1. **加载双臂配置**：选择双臂机器人类型
2. **选择控制模式**：
   - **绝对位姿模式**：控制绝对位姿，保持相对位姿不变
   - **相对位姿模式**：控制相对位姿，保持绝对位姿不变
   - **机器人1模式**：单独控制机器人1的末端执行器
   - **机器人2模式**：单独控制机器人2的末端执行器
3. **协同操作**：两个机械臂可以独立或协同工作

### 关节角度管理

- **复制关节角度**：点击"复制关节角度"按钮，将当前角度复制到剪贴板
- **导入关节角度**：点击"从剪贴板导入"按钮，从剪贴板读取并应用角度
- **格式示例**：
  - 单臂：`[0.52356, 0, 0, 1.04712, 0, 0, 0]`
  - 双臂：
    ```
    robot1=[0.52356, 0, 0, 1.04712, 0, 0, 0]
    robot2=[-0.52356, 0, 0, -1.04712, 0, 0, 0]
    ```

### 角度单位切换

- 点击"切换为弧度/角度"按钮在度（°）和弧度（rad）之间切换
- 默认单位为弧度
- 切换后所有显示和输入都会自动转换

### URDF 模型加载

1. **拖拽加载**：将 URDF 文件拖拽到浏览器窗口
2. **自动解析**：系统会自动解析 URDF 并提取关节链
3. **MDH 参数**：可以查看从 URDF 提取的 MDH 参数

## 🏗️ 项目结构

```
dq_vizlab/
├── public/
│   ├── robots/          # 机器人配置文件（URDF, YAML）
│   └── wasm/            # 编译后的 WASM 模块
├── src/
│   ├── main.js          # 主程序入口
│   ├── visualizer.js    # 3D 可视化和交互逻辑
│   ├── dqrobotics.js    # DQ Robotics WASM 接口
│   ├── urdf-loader.js   # URDF 加载器
│   ├── urdf-to-mdh.js   # URDF 到 MDH 转换
│   └── styles.css       # 样式文件
├── wasm/                # WASM 源代码（C++）
├── eigen/               # Eigen 数学库（子模块）
├── dqrobotics-cpp/      # DQ Robotics C++ 库（子模块）
├── build.sh             # WASM 构建脚本
├── vite.config.js       # Vite 配置
└── package.json         # 项目配置
```

## 🔧 开发说明

### WASM 构建流程

1. `build.sh` 脚本会：
   - 检查 Emscripten 环境
   - 下载/检查 Eigen 库
   - 使用 CMake 配置构建
   - 编译 C++ 代码为 WASM
   - 将输出安装到 `public/wasm/`

### 核心算法

- **正运动学**：使用 DQ Robotics 的 `fkm()` 方法
- **逆运动学**：基于伪逆雅可比迭代法
- **位姿表示**：对偶四元数（Dual Quaternion）
- **坐标变换**：支持局部坐标系和世界坐标系

### 点动控制增量

- **平移增量**：`stepSize = 0.01` (1cm)
- **旋转增量**：`angle = π/128` (约 1.4°)

可在 `src/visualizer.js` 中修改这些值。

## 🌐 部署

### GitHub Pages

项目已配置 GitHub Actions 自动部署到 GitHub Pages：

1. 推送代码到 `main` 分支
2. GitHub Actions 自动构建并部署到 `gh-pages` 分支
3. 在仓库设置中启用 GitHub Pages

### 手动部署

```bash
npm run build
# 将 dist/ 目录内容部署到静态服务器
```

## 📝 许可证

MIT License

## 🙏 致谢

- [DQ Robotics](https://github.com/dqrobotics) - 对偶四元数机器人学库
- [Eigen](https://eigen.tuxfamily.org/) - C++ 线性代数库
- [Three.js](https://threejs.org/) - 3D 图形库
- [Emscripten](https://emscripten.org/) - WebAssembly 工具链

## 📧 联系方式

如有问题或建议，请提交 Issue 或 Pull Request。

