/**
 * 机器人 3D 可视化器 - 使用 Three.js 和 URDF（Z-up 坐标系）
 */

import * as THREE from 'three';
import URDFLoader from 'urdf-loader';

export class RobotVisualizer {
    constructor(canvas) {
        this.canvas = canvas;
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.robot = null;
        this.urdfRobot = null;
        this.endEffector = null;
        this.useURDF = true;
        
        // 双臂支持
        this.dualArmMode = false;
        this.robot1 = null;
        this.robot1URDF = null;
        this.robot1EndEffector = null;
        this.robot2 = null;
        this.robot2URDF = null;
        this.robot2EndEffector = null;
        this.absolutePoseMarker = null;  // 绝对位姿标记
        this.relativePoseLine = null;    // 相对位姿连线
        
        // 坐标系指示器
        this.axesScene = null;
        this.axesCamera = null;
        
        // MDH坐标系显示
        this.mdhFramesGroup = null;
        this.showMDHFrames = false;
        this.lastMDHFrameOrigin = null; // 最后一个MDH坐标系的原点位置
        
        // 鼠标控制
        this.isLeftDragging = false;  // 左键拖动标志（旋转视角）
        this.isRightDragging = false; // 右键拖动标志（平移位置）
        this.previousMousePosition = { x: 0, y: 0 };
        this.cameraRotation = { 
            x: Math.PI / 2.2,    // 约82度 - 几乎水平，稍微从上往下看
            y: 0                 // 0度 - 沿X轴正方向观察
        };
        this.cameraDistance = 2.0;  // 适中距离
        this.cameraPan = { x: 0, y: 0, z: 0 }; // 相机平移偏移
        
        // 拖动末端执行器模式
        this.dragEndEffectorMode = false;  // 拖动模式开关
        this.isDraggingEndEffector = false;  // 是否正在拖动末端执行器
        this.raycaster = new THREE.Raycaster();  // 用于鼠标拾取
        this.mouse = new THREE.Vector2();  // 鼠标位置
        this.dragPlane = null;  // 拖动平面
        this.dragStartPosition = null;  // 拖动起始位置
        this.dragCurrentPosition = null;  // 拖动当前位置
        
        // MoveIt 风格的拖动箭头
        this.endEffectorGizmo = null;  // 拖动控制组（包含平移和旋转箭头）
        this.translationArrows = { 
            x: { positive: null, negative: null }, 
            y: { positive: null, negative: null }, 
            z: { positive: null, negative: null } 
        };  // 平移箭头（正负方向）
        this.rotationArrows = { x: null, y: null, z: null };  // 旋转箭头
        this.selectedArrow = null;  // 当前选中的箭头
        this.arrowType = null;  // 'translation' 或 'rotation'
        this.arrowAxis = null;  // 'x', 'y', 或 'z'
    }
    
    /**
     * 初始化 Three.js 场景
     */
    async init() {
        // 创建场景
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x2d2d3e); // 调亮背景色（从 0x1a1a2e 改为 0x2d2d3e）
        
        // 确保canvas尺寸已正确设置
        if (this.canvas.clientWidth === 0 || this.canvas.clientHeight === 0) {
            // 如果canvas尺寸为0，等待一帧再继续
            await new Promise(resolve => requestAnimationFrame(resolve));
        }
        
        // 创建相机
        const aspect = this.canvas.clientWidth / this.canvas.clientHeight || 1;
        this.camera = new THREE.PerspectiveCamera(60, aspect, 0.1, 100);
        
        // 创建渲染器
        this.renderer = new THREE.WebGLRenderer({
            canvas: this.canvas,
            antialias: true,
            alpha: false
        });
        this.renderer.setSize(this.canvas.clientWidth || window.innerWidth, this.canvas.clientHeight || window.innerHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        
        // 更新相机位置（在渲染器设置之后）
        this.updateCameraPosition();
        
        // 确保相机和渲染器同步更新
        this.camera.aspect = this.canvas.clientWidth / this.canvas.clientHeight || 1;
        this.camera.updateProjectionMatrix();
        
        // 添加光源（增强亮度）
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.9); // 从 0.7 增加到 0.9
        this.scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8); // 从 0.6 增加到 0.8
        directionalLight.position.set(3, 3, 5);
        directionalLight.castShadow = true;
        this.scene.add(directionalLight);
        
        const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.5); // 从 0.3 增加到 0.5
        directionalLight2.position.set(-3, -3, 2);
        this.scene.add(directionalLight2);
        
        // 添加地面网格（Z-up: 网格在 XY 平面，调亮网格颜色）
        const gridHelper = new THREE.GridHelper(5, 20, 0x888888, 0x555555); // 从 0x666666/0x333333 调亮到 0x888888/0x555555
        gridHelper.rotation.x = Math.PI / 2; // 旋转到 XY 平面
        this.scene.add(gridHelper);
        
        // 添加世界坐标轴（Z-up）
        const worldAxes = new THREE.AxesHelper(0.5);
        this.scene.add(worldAxes);
        
        // 创建机器人
        await this.createRobot();
        
        // 创建末端执行器标记
        this.createEndEffectorMarker();
        
        // 创建左上角坐标系指示器
        this.createAxesIndicator();
        
        // 设置事件监听
        this.setupEventListeners();
        
        // 确保首次渲染时相机位置正确（在所有对象创建后）
        // 等待一帧确保DOM完全渲染
        await new Promise(resolve => requestAnimationFrame(resolve));
        
        // 再次检查并更新canvas尺寸
        const width = this.canvas.clientWidth || window.innerWidth;
        const height = this.canvas.clientHeight || window.innerHeight;
        if (width > 0 && height > 0) {
            this.renderer.setSize(width, height);
            this.camera.aspect = width / height;
            this.camera.updateProjectionMatrix();
        }
        
        // 更新相机位置
        this.updateCameraPosition();
        
        // 强制更新相机投影矩阵
        this.camera.updateProjectionMatrix();
        
        // 立即渲染一次，确保初始状态正确
        this.renderer.render(this.scene, this.camera);
        
        // 再渲染一次坐标系指示器
        if (this.axesScene && this.axesCamera) {
            const size = 120;
            this.renderer.autoClear = false;
            this.renderer.clearDepth();
            this.renderer.setViewport(20, height - size - 20, size, size);
            this.renderer.setScissor(20, height - size - 20, size, size);
            this.renderer.setScissorTest(true);
            this.renderer.render(this.axesScene, this.axesCamera);
            this.renderer.setScissorTest(false);
            this.renderer.setViewport(0, 0, width, height);
            this.renderer.autoClear = true;
        }
        
        // 开始渲染循环
        this.animate();
        
        console.log('✓ 可视化器初始化完成（Z-up 坐标系）');
    }
    
    /**
     * 创建机器人模型
     */
    async createRobot() {
        // 尝试加载 URDF
        if (this.useURDF) {
            try {
                const success = await this.loadURDFRobot();
                if (success) {
                    console.log('✓ 使用 URDF 模型');
                    return;
                }
            } catch (error) {
                console.log('↓ URDF 加载失败，使用简化模型');
            }
        }
        
        // 创建简化几何模型
        this.createSimplifiedRobot();
    }
    
    /**
     * 加载 URDF 机器人模型
     */
    async loadURDFRobot() {
        try {
            const loader = new URDFLoader();
            const baseUrl = import.meta.env.BASE_URL || '/';
            loader.packages = {
                'franka_description': `${baseUrl}robots/franka`
            };
            
            this.urdfRobot = await new Promise((resolve, reject) => {
                loader.load(
                    `${baseUrl}robots/franka/panda_simplified.urdf`,
                    (robot) => {
                        console.log('✓ URDF 加载成功');
                        console.log('关节数:', Object.keys(robot.joints).length);
                        resolve(robot);
                    },
                    undefined,
                    (error) => {
                        console.error('URDF 加载失败:', error);
                        reject(error);
                    }
                );
            });
            
            // URDF 模型保持原始 Y-up，直接添加到场景
            this.scene.add(this.urdfRobot);
            return true;
            
        } catch (error) {
            console.warn('无法加载 URDF:', error);
            return false;
        }
    }
    
    /**
     * 创建简化的几何机器人模型
     */
    createSimplifiedRobot() {
        this.robot = new THREE.Group();
        
        // 材质
        const linkMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xeeeeee,
            shininess: 60
        });
        const jointMaterial = new THREE.MeshPhongMaterial({ 
            color: 0x666666,
            shininess: 80
        });
        
        // 基座（Z-up坐标系，所以使用Z轴）
        const base = new THREE.Mesh(
            new THREE.CylinderGeometry(0.12, 0.15, 0.1, 32),
            jointMaterial
        );
        base.rotation.x = Math.PI / 2; // 旋转使其Z轴向上
        base.position.z = 0.05;
        this.robot.add(base);
        
        // 立柱
        const column = new THREE.Mesh(
            new THREE.CylinderGeometry(0.08, 0.08, 0.3, 16),
            linkMaterial
        );
        column.rotation.x = Math.PI / 2;
        column.position.z = 0.25;
        this.robot.add(column);
        
        // 上臂
        const upperArm = new THREE.Mesh(
            new THREE.BoxGeometry(0.08, 0.08, 0.3),
            linkMaterial
        );
        upperArm.position.set(0.1, 0, 0.5);
        this.robot.add(upperArm);
        
        // 前臂
        const forearm = new THREE.Mesh(
            new THREE.BoxGeometry(0.06, 0.06, 0.25),
            linkMaterial
        );
        forearm.position.set(0.25, 0, 0.6);
        this.robot.add(forearm);
        
        this.scene.add(this.robot);
        console.log('✓ 简化模型已创建');
    }
    
    /**
     * 创建末端执行器标记
     */
    createEndEffectorMarker() {
        const geometry = new THREE.SphereGeometry(0.02, 32, 32);
        const material = new THREE.MeshPhongMaterial({ 
            color: 0xfbbf24,
            emissive: 0xfbbf24,
            emissiveIntensity: 0.5,
            transparent: true,
            opacity: 0.9,
            depthTest: false,  // 不进行深度测试，始终显示在最前面
            depthWrite: false  // 不写入深度缓冲
        });
        this.endEffector = new THREE.Mesh(geometry, material);
        this.endEffector.position.set(0.3, 0, 0.6);
        this.endEffector.renderOrder = 999;  // 设置高渲染优先级，最后渲染
        
        // 添加末端坐标轴（也设置为始终显示在前面）
        const endAxes = new THREE.AxesHelper(0.1);
        endAxes.material.depthTest = false;
        endAxes.material.depthWrite = false;
        endAxes.renderOrder = 999;
        this.endEffector.add(endAxes);
        
        this.scene.add(this.endEffector);
        console.log('✓ 末端执行器标记已创建（始终显示在最前面）');
    }
    
    /**
     * 创建左上角坐标系指示器
     */
    createAxesIndicator() {
        this.axesScene = new THREE.Scene();
        
        // 创建相机
        this.axesCamera = new THREE.OrthographicCamera(-1.5, 1.5, 1.5, -1.5, 0.1, 10);
        this.axesCamera.position.set(3, 3, 3);
        this.axesCamera.lookAt(0, 0, 0);
        
        // 创建坐标轴
        const axesHelper = new THREE.AxesHelper(1);
        this.axesScene.add(axesHelper);
        
        // 创建文字标签
        const createTextSprite = (text, color, position) => {
            const canvas = document.createElement('canvas');
            const context = canvas.getContext('2d');
            canvas.width = 128;
            canvas.height = 128;
            
            context.fillStyle = color;
            context.font = 'Bold 90px Arial';
            context.textAlign = 'center';
            context.textBaseline = 'middle';
            context.fillText(text, 64, 64);
            
            const texture = new THREE.CanvasTexture(canvas);
            const spriteMaterial = new THREE.SpriteMaterial({ 
                map: texture,
                depthTest: false
            });
            const sprite = new THREE.Sprite(spriteMaterial);
            sprite.position.copy(position);
            sprite.scale.set(0.4, 0.4, 1);
            
            return sprite;
        };
        
        // 添加 XYZ 标签
        this.axesScene.add(createTextSprite('X', '#ef4444', new THREE.Vector3(1.2, 0, 0)));
        this.axesScene.add(createTextSprite('Y', '#10b981', new THREE.Vector3(0, 1.2, 0)));
        this.axesScene.add(createTextSprite('Z', '#3b82f6', new THREE.Vector3(0, 0, 1.2)));
        
        console.log('✓ 坐标系指示器已创建');
    }
    
    /**
     * 更新相机位置
     */
    updateCameraPosition() {
        const theta = this.cameraRotation.y;
        const phi = this.cameraRotation.x;
        
        const x = this.cameraDistance * Math.sin(phi) * Math.cos(theta);
        const y = this.cameraDistance * Math.sin(phi) * Math.sin(theta);
        const z = this.cameraDistance * Math.cos(phi);
        
        // 应用相机位置和平移偏移
        this.camera.position.set(x + this.cameraPan.x, y + this.cameraPan.y, z + this.cameraPan.z);
        
        // 计算目标点（也应用平移偏移）
        const targetX = this.cameraPan.x;
        const targetY = this.cameraPan.y;
        const targetZ = 0.5 + this.cameraPan.z;
        this.camera.lookAt(targetX, targetY, targetZ);
        this.camera.up.set(0, 0, 1); // Z轴向上
        
        // 同步坐标系指示器
        if (this.axesCamera) {
            this.axesCamera.position.copy(this.camera.position).normalize().multiplyScalar(4);
            this.axesCamera.lookAt(0, 0, 0);
            this.axesCamera.up.set(0, 0, 1);
        }
    }
    
    /**
     * 设置事件监听器
     */
    setupEventListeners() {
        // 窗口大小调整
        window.addEventListener('resize', () => {
            const width = this.canvas.clientWidth;
            const height = this.canvas.clientHeight;
            
            this.camera.aspect = width / height;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(width, height);
        });
        
        // 鼠标拖动
        // 左键：旋转视角
        // 右键：平移位置
        this.canvas.addEventListener('mousedown', (e) => {
            // 如果拖动末端执行器模式开启，优先处理拖动
            if (this.dragEndEffectorMode && e.button === 0) {
                this.handleDragEndEffectorMouseDown(e);
                if (this.isDraggingEndEffector) {
                    return; // 如果正在拖动末端执行器，不处理相机旋转
                }
            }
            
            if (e.button === 0) {
                // 左键：旋转视角
                this.isLeftDragging = true;
                this.previousMousePosition = { x: e.clientX, y: e.clientY };
            } else if (e.button === 2) {
                // 右键：平移位置
                this.isRightDragging = true;
                this.previousMousePosition = { x: e.clientX, y: e.clientY };
                // 阻止默认的右键菜单
                e.preventDefault();
            }
        });
        
        this.canvas.addEventListener('mousemove', (e) => {
            // 如果拖动末端执行器模式开启，优先处理拖动
            if (this.dragEndEffectorMode && this.isDraggingEndEffector) {
                this.handleDragEndEffectorMouseMove(e);
                return; // 如果正在拖动末端执行器，不处理相机旋转
            }
            
            const deltaX = e.clientX - this.previousMousePosition.x;
            const deltaY = e.clientY - this.previousMousePosition.y;
            
            if (this.isLeftDragging) {
                // 左键拖动：旋转视角
                this.cameraRotation.y += deltaX * 0.01;
                this.cameraRotation.x -= deltaY * 0.01;
                
                // 限制角度
                this.cameraRotation.x = Math.max(0.1, Math.min(Math.PI - 0.1, this.cameraRotation.x));
                
                this.updateCameraPosition();
            } else if (this.isRightDragging) {
                // 右键拖动：平移位置
                // 计算相机右方向和上方向
                const right = new THREE.Vector3();
                const up = new THREE.Vector3();
                const forward = new THREE.Vector3();
                
                forward.subVectors(
                    new THREE.Vector3(this.cameraPan.x, this.cameraPan.y, 0.5 + this.cameraPan.z),
                    this.camera.position
                ).normalize();
                
                right.crossVectors(forward, new THREE.Vector3(0, 0, 1)).normalize();
                up.crossVectors(right, forward).normalize();
                
                // 根据鼠标移动量平移相机
                const panSpeed = this.cameraDistance * 0.001;
                this.cameraPan.x -= right.x * deltaX * panSpeed;
                this.cameraPan.y -= right.y * deltaX * panSpeed;
                this.cameraPan.z += up.z * deltaY * panSpeed;
                
                this.updateCameraPosition();
            }
            
            if (this.isLeftDragging || this.isRightDragging) {
                this.previousMousePosition = { x: e.clientX, y: e.clientY };
            }
        });
        
        this.canvas.addEventListener('mouseup', (e) => {
            // 如果拖动末端执行器模式开启，优先处理拖动
            if (this.dragEndEffectorMode && e.button === 0) {
                this.handleDragEndEffectorMouseUp(e);
            }
            
            if (e.button === 0) {
                this.isLeftDragging = false;
            } else if (e.button === 2) {
                this.isRightDragging = false;
            }
        });
        
        this.canvas.addEventListener('mouseleave', () => {
            this.isLeftDragging = false;
            this.isRightDragging = false;
            this.isDraggingEndEffector = false;
        });
        
        // 阻止右键菜单（在canvas上）
        this.canvas.addEventListener('contextmenu', (e) => {
            e.preventDefault();
        });
        
        // 滚轮缩放
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            this.cameraDistance *= (1 + e.deltaY * 0.001);
            this.cameraDistance = Math.max(0.5, Math.min(10, this.cameraDistance));
            this.updateCameraPosition();
        }, { passive: false });
    }
    
    /**
     * 动画循环
     */
    animate() {
        requestAnimationFrame(() => this.animate());
        
        // 渲染主场景
        this.renderer.render(this.scene, this.camera);
        
        // 渲染左上角坐标系指示器
        if (this.axesScene && this.axesCamera) {
            this.renderer.autoClear = false;
            this.renderer.clearDepth();
            
            const width = this.canvas.clientWidth;
            const height = this.canvas.clientHeight;
            const size = 120;
            
            this.renderer.setViewport(20, height - size - 20, size, size);
            this.renderer.setScissor(20, height - size - 20, size, size);
            this.renderer.setScissorTest(true);
            
            this.renderer.render(this.axesScene, this.axesCamera);
            
            this.renderer.setViewport(0, 0, width, height);
            this.renderer.setScissorTest(false);
            this.renderer.autoClear = true;
        }
    }
    
    /**
     * 加载自定义 URDF 模型（单臂或双臂）
     * @param {Object} robot - urdf-loader 加载的机器人对象
     * @param {Object} robot2 - 可选的第二个机器人对象（双臂模式，two_urdf格式）或null（single_urdf格式）
     * @param {Object} config - 配置对象，包含双臂模式和基座位姿等
     * @param {string} dualArmType - 双臂格式类型：'two_urdf' 或 'single_urdf'
     */
    async loadCustomURDF(robot, robot2 = null, config = null, dualArmType = 'two_urdf') {
        try {
            console.log('  → 开始加载自定义 URDF 到场景...');
            
            // 检查是否为双臂模式
            const isDualArm = config?.dual_arm === true;
            this.dualArmMode = isDualArm;
            this.dualArmType = dualArmType; // 存储双臂格式类型
            
            if (isDualArm) {
                console.log(`  → 双臂模式：格式=${dualArmType}`);
                return await this.loadDualArmURDF(robot, robot2, config, dualArmType);
            } else {
                console.log('  → 单臂模式：加载单个机器人');
                return await this.loadSingleArmURDF(robot, config);
            }
        } catch (error) {
            console.error('❌ 加载自定义 URDF 失败:', error);
            console.error('错误堆栈:', error.stack);
            return false;
        }
    }
    
    /**
     * 加载单臂 URDF 模型
     * @param {Object} robot - urdf-loader 加载的机器人对象
     * @param {Object} config - 配置对象，包含基座位姿等
     */
    async loadSingleArmURDF(robot, config = null) {
        // 移除旧的机器人模型
        if (this.urdfRobot) {
            this.scene.remove(this.urdfRobot);
            this.urdfRobot = null;
        }
        if (this.robot) {
            this.scene.remove(this.robot);
            this.robot = null;
        }
        
        // 移除双臂相关对象
        if (this.robot1URDF) {
            this.scene.remove(this.robot1URDF);
            this.robot1URDF = null;
        }
        if (this.robot2URDF) {
            this.scene.remove(this.robot2URDF);
            this.robot2URDF = null;
        }
        
        // 移除双臂末端执行器标记
        if (this.robot1EndEffector) {
            this.scene.remove(this.robot1EndEffector);
            this.robot1EndEffector = null;
        }
        if (this.robot2EndEffector) {
            this.scene.remove(this.robot2EndEffector);
            this.robot2EndEffector = null;
        }
        
        // 移除双臂可视化元素（绝对位姿、相对位姿）
        if (this.absolutePoseMarker) {
            this.scene.remove(this.absolutePoseMarker);
            this.absolutePoseMarker = null;
        }
        if (this.relativePoseLine) {
            this.scene.remove(this.relativePoseLine);
            this.relativePoseLine = null;
        }
        
        // 设置双臂模式为false
        this.dualArmMode = false;
        
        // 应用基座位姿（如果配置中有）
        // 优先从 mdh_extraction.base_pose 读取，如果不存在则从 config.base_pose 读取
        let basePose = config?.mdh_extraction?.base_pose || config?.base_pose;
        if (basePose && Array.isArray(basePose) && basePose.length >= 3) {
            console.log('  设置单臂基座位姿:', basePose);
            if (basePose.length === 8) {
                // 双四元数格式 [qw, qx, qy, qz, qw', qx', qy', qz']
                // 使用 DQModule 提取旋转和平移
                if (window.DQModule && window.DQModule.DQWrapper) {
                    try {
                        console.log(`    原始双四元数: [${basePose.map(v => v.toFixed(6)).join(', ')}]`);
                        
                        // 创建双四元数并归一化（必须归一化才能提取旋转和平移）
                        let dq = window.DQModule.DQWrapper.createFromArray(basePose);
                        dq = window.DQModule.DQWrapper.normalize(dq);
                        
                        const rotation = window.DQModule.DQWrapper.getRotation(dq);
                        const translation = window.DQModule.DQWrapper.getTranslation(dq);
                        
                        console.log(`    提取的旋转: [${rotation.map(v => v.toFixed(6)).join(', ')}]`);
                        console.log(`    提取的平移: [${translation.map(v => v.toFixed(6)).join(', ')}]`);
                        
                        // rotation: [qw, qx, qy, qz]
                        // translation: [w, x, y, z] (w=0, xyz是位置)
                        const [qw, qx, qy, qz] = rotation;
                        const [, tx, ty, tz] = translation;
                        
                        robot.position.set(tx || 0, ty || 0, tz || 0);
                        robot.quaternion.set(qx, qy, qz, qw);
                        console.log(`    应用的位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                        console.log(`    应用的旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                    } catch (error) {
                        console.warn('  无法从双四元数提取位姿，回退到前4个元素作为旋转:', error);
                        // 回退：假设前4个是旋转，后4个包含平移信息
                        const [qw, qx, qy, qz] = basePose.slice(0, 4);
                        robot.quaternion.set(qx, qy, qz, qw);
                        console.log(`    旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                    }
                } else {
                    console.warn('  DQModule 不可用，无法从双四元数提取位姿');
                }
            } else if (basePose.length === 7) {
                // 完整位姿 [qw, qx, qy, qz, tx, ty, tz]
                const [qw, qx, qy, qz, tx, ty, tz] = basePose;
                robot.position.set(tx || 0, ty || 0, tz || 0);
                // Three.js 四元数顺序是 (x, y, z, w)
                robot.quaternion.set(qx, qy, qz, qw);
                console.log(`    位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                console.log(`    旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
            } else if (basePose.length >= 3) {
                // 仅平移 [tx, ty, tz] 或 [tx, ty, tz, ...]
                const [tx, ty, tz] = basePose;
                robot.position.set(tx || 0, ty || 0, tz || 0);
                console.log(`    位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
            }
        }
        
        // 添加新模型
        this.urdfRobot = robot;
        this.scene.add(robot);
        
        // 重新创建单臂末端执行器标记（如果之前没有）
        if (!this.endEffector) {
            this.createEndEffectorMarker();
        }
        
        console.log('✓ 单臂 URDF 加载完成');
        return true;
    }
    
    /**
     * 加载双臂 URDF 模型
     * @param {Object} robot1 - 第一个机器人对象（two_urdf格式）或共享URDF对象（single_urdf格式）
     * @param {Object} robot2 - 第二个机器人对象（two_urdf格式）或null（single_urdf格式）
     * @param {Object} config - 配置对象
     * @param {string} dualArmType - 双臂格式类型：'two_urdf' 或 'single_urdf'
     */
    async loadDualArmURDF(robot1, robot2, config, dualArmType = 'two_urdf') {
        // 移除旧的机器人模型
        if (this.urdfRobot) {
            this.scene.remove(this.urdfRobot);
            this.urdfRobot = null;
        }
        if (this.robot) {
            this.scene.remove(this.robot);
            this.robot = null;
        }
        
        // 移除旧的末端执行器
        if (this.endEffector) {
            this.scene.remove(this.endEffector);
            this.endEffector = null;
        }
        
        if (dualArmType === 'single_urdf') {
            // 单URDF格式：只添加一个URDF对象（包含两条链）
            console.log('  → 单URDF格式：只添加一个共享URDF模型');
            this.robot1URDF = robot1;
            this.robot2URDF = robot1; // 两个机器人使用同一个URDF对象
            
            // 对于single_urdf格式，通常不需要设置base_pose（因为两条链共享base_link）
            // 但如果配置中有偏移，可以应用
            const basePose1 = config?.robot1?.base_pose || [0, 0, 0];
            if (Array.isArray(basePose1) && basePose1.some(v => v !== 0)) {
                console.log('  应用机器人1基座偏移:', basePose1);
                if (basePose1.length >= 3) {
                    const [tx, ty, tz] = basePose1;
                    robot1.position.set(tx || 0, ty || 0, tz || 0);
                }
            }
            
            this.scene.add(robot1);
            console.log('  ✓ 共享URDF模型已添加到场景，场景对象数:', this.scene.children.length);
        } else {
            // 双URDF格式：添加两个独立的URDF对象
            console.log('  → 双URDF格式：添加两个独立的URDF模型');
            
            // 设置机器人1
            this.robot1URDF = robot1;
            const basePose1 = config?.robot1?.base_pose || [0, 0, 0];
            console.log('  设置机器人1基座位姿:', basePose1);
            if (Array.isArray(basePose1)) {
                if (basePose1.length === 8) {
                    // 双四元数格式 [qw, qx, qy, qz, qw', qx', qy', qz']
                    if (window.DQModule && window.DQModule.DQWrapper) {
                        try {
                            console.log(`  机器人1原始双四元数: [${basePose1.map(v => v.toFixed(6)).join(', ')}]`);
                            
                            // 创建双四元数并归一化（必须归一化才能提取旋转和平移）
                            let dq = window.DQModule.DQWrapper.createFromArray(basePose1);
                            dq = window.DQModule.DQWrapper.normalize(dq);
                            
                            const rotation = window.DQModule.DQWrapper.getRotation(dq);
                            const translation = window.DQModule.DQWrapper.getTranslation(dq);
                            
                            console.log(`  机器人1提取的旋转: [${rotation.map(v => v.toFixed(6)).join(', ')}]`);
                            console.log(`  机器人1提取的平移: [${translation.map(v => v.toFixed(6)).join(', ')}]`);
                            
                            const [qw, qx, qy, qz] = rotation;
                            const [, tx, ty, tz] = translation;
                            
                            robot1.position.set(tx || 0, ty || 0, tz || 0);
                            robot1.quaternion.set(qx, qy, qz, qw);
                            console.log(`  机器人1应用的位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                            console.log(`  机器人1应用的旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                        } catch (error) {
                            console.warn('  无法从双四元数提取位姿:', error);
                        }
                    } else {
                        console.warn('  DQModule 不可用，无法从双四元数提取位姿');
                    }
                } else if (basePose1.length === 7) {
                    // 完整位姿 [qw, qx, qy, qz, tx, ty, tz]
                    const [qw, qx, qy, qz, tx, ty, tz] = basePose1;
                    robot1.position.set(tx || 0, ty || 0, tz || 0);
                    robot1.quaternion.set(qx, qy, qz, qw);
                    console.log(`  机器人1位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                    console.log(`  机器人1旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                } else if (basePose1.length >= 3) {
                    // 仅平移 [tx, ty, tz] 或 [tx, ty, tz, ...]
                    const [tx, ty, tz] = basePose1;
                    robot1.position.set(tx || 0, ty || 0, tz || 0);
                    console.log(`  机器人1位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                }
            }
            this.scene.add(robot1);
            console.log('  ✓ 机器人1已添加到场景，场景对象数:', this.scene.children.length);
            
            // 设置机器人2
            this.robot2URDF = robot2;
            const basePose2 = config?.robot2?.base_pose || [0, 0, 0];
            console.log('  设置机器人2基座位姿:', basePose2);
            if (Array.isArray(basePose2)) {
                if (basePose2.length === 8) {
                    // 双四元数格式 [qw, qx, qy, qz, qw', qx', qy', qz']
                    if (window.DQModule && window.DQModule.DQWrapper) {
                        try {
                            console.log(`  机器人2原始双四元数: [${basePose2.map(v => v.toFixed(6)).join(', ')}]`);
                            
                            // 创建双四元数并归一化（必须归一化才能提取旋转和平移）
                            let dq = window.DQModule.DQWrapper.createFromArray(basePose2);
                            dq = window.DQModule.DQWrapper.normalize(dq);
                            
                            const rotation = window.DQModule.DQWrapper.getRotation(dq);
                            const translation = window.DQModule.DQWrapper.getTranslation(dq);
                            
                            console.log(`  机器人2提取的旋转: [${rotation.map(v => v.toFixed(6)).join(', ')}]`);
                            console.log(`  机器人2提取的平移: [${translation.map(v => v.toFixed(6)).join(', ')}]`);
                            
                            const [qw, qx, qy, qz] = rotation;
                            const [, tx, ty, tz] = translation;
                            
                            robot2.position.set(tx || 0, ty || 0, tz || 0);
                            robot2.quaternion.set(qx, qy, qz, qw);
                            console.log(`  机器人2应用的位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                            console.log(`  机器人2应用的旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                        } catch (error) {
                            console.warn('  无法从双四元数提取位姿:', error);
                        }
                    } else {
                        console.warn('  DQModule 不可用，无法从双四元数提取位姿');
                    }
                } else if (basePose2.length === 7) {
                    // 完整位姿 [qw, qx, qy, qz, tx, ty, tz]
                    const [qw, qx, qy, qz, tx, ty, tz] = basePose2;
                    robot2.position.set(tx || 0, ty || 0, tz || 0);
                    robot2.quaternion.set(qx, qy, qz, qw);
                    console.log(`  机器人2位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                    console.log(`  机器人2旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                } else if (basePose2.length >= 3) {
                    // 仅平移 [tx, ty, tz] 或 [tx, ty, tz, ...]
                    const [tx, ty, tz] = basePose2;
                    robot2.position.set(tx || 0, ty || 0, tz || 0);
                    console.log(`  机器人2位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                }
            }
            this.scene.add(robot2);
            console.log('  ✓ 机器人2已添加到场景，场景对象数:', this.scene.children.length);
            console.log('  场景中的所有对象:', this.scene.children.map(c => c.name || c.type || c.constructor.name));
        }
        
        // 创建末端执行器标记
        this.createDualArmEndEffectors(config);
        
        // 创建绝对位姿和相对位姿可视化
        this.createDualArmVisualizations(config);
        
        console.log('✓ 双臂 URDF 加载完成，最终场景对象数:', this.scene.children.length);
        return true;
    }
    
    /**
     * 创建双臂末端执行器标记
     */
    createDualArmEndEffectors(config) {
        // 先清理旧的末端执行器标记
        if (this.robot1EndEffector) {
            this.scene.remove(this.robot1EndEffector);
            this.robot1EndEffector = null;
        }
        if (this.robot2EndEffector) {
            this.scene.remove(this.robot2EndEffector);
            this.robot2EndEffector = null;
        }
        
        const visConfig = config?.visualization || {};
        const size = visConfig.end_effector_size || 0.02;
        
        // 处理颜色（可能是字符串 "#3b82f6" 或数字 0x3b82f6）
        let color1 = visConfig.robot1_color || 0x3b82f6;
        let color2 = visConfig.robot2_color || 0x10b981;
        
        if (typeof color1 === 'string' && color1.startsWith('#')) {
            color1 = parseInt(color1.substring(1), 16);
        }
        if (typeof color2 === 'string' && color2.startsWith('#')) {
            color2 = parseInt(color2.substring(1), 16);
        }
        
        // 机器人1末端执行器
        const geometry1 = new THREE.SphereGeometry(size, 32, 32);
        const material1 = new THREE.MeshPhongMaterial({ 
            color: color1,
            emissive: color1,
            emissiveIntensity: 0.5,
            transparent: true,
            opacity: 0.9
        });
        material1.depthTest = false;
        material1.depthWrite = false;
        this.robot1EndEffector = new THREE.Mesh(geometry1, material1);
        this.robot1EndEffector.renderOrder = 999;
        const axes1 = new THREE.AxesHelper(size * 2);
        if (axes1.material) {
            axes1.material.depthTest = false;
            axes1.material.depthWrite = false;
            axes1.material.transparent = true;
            axes1.material.opacity = 0.95;
        }
        this.robot1EndEffector.add(axes1);
        this.scene.add(this.robot1EndEffector);
        
        // 机器人2末端执行器
        const geometry2 = new THREE.SphereGeometry(size, 32, 32);
        const material2 = new THREE.MeshPhongMaterial({ 
            color: color2,
            emissive: color2,
            emissiveIntensity: 0.5,
            transparent: true,
            opacity: 0.9
        });
        material2.depthTest = false;
        material2.depthWrite = false;
        this.robot2EndEffector = new THREE.Mesh(geometry2, material2);
        this.robot2EndEffector.renderOrder = 999;
        const axes2 = new THREE.AxesHelper(size * 2);
        if (axes2.material) {
            axes2.material.depthTest = false;
            axes2.material.depthWrite = false;
            axes2.material.transparent = true;
            axes2.material.opacity = 0.95;
        }
        this.robot2EndEffector.add(axes2);
        this.scene.add(this.robot2EndEffector);
    }
    
    /**
     * 创建双臂可视化元素（绝对位姿、相对位姿）
     */
    createDualArmVisualizations(config) {
        // 先清理旧的可视化元素
        if (this.absolutePoseMarker) {
            this.scene.remove(this.absolutePoseMarker);
            this.absolutePoseMarker = null;
        }
        if (this.relativePoseLine) {
            this.scene.remove(this.relativePoseLine);
            this.relativePoseLine = null;
        }
        
        const visConfig = config?.visualization || {};
        
        // 绝对位姿标记（两个末端执行器的中点）
        if (visConfig.show_absolute_pose !== false) {
            let color = visConfig.absolute_pose_color || 0xfbbf24;
            if (typeof color === 'string' && color.startsWith('#')) {
                color = parseInt(color.substring(1), 16);
            }
            
            // 创建标记组
            this.absolutePoseMarker = new THREE.Group();
            // 确保Group的scale始终为(1,1,1)，避免变形
            this.absolutePoseMarker.scale.set(1, 1, 1);
            
            // 创建球体
            const sphereGeometry = new THREE.SphereGeometry(0.02, 16, 16);
            const sphereMaterial = new THREE.MeshPhongMaterial({ 
                color: color,
                emissive: color,
                emissiveIntensity: 0.7,
                transparent: true,
                opacity: 0.9
            });
            sphereMaterial.depthTest = false;
            sphereMaterial.depthWrite = false;
            const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
            sphere.renderOrder = 998;
            // 确保球体scale为(1,1,1)
            sphere.scale.set(1, 1, 1);
            this.absolutePoseMarker.add(sphere);
            
            // 添加坐标轴（稍大一些以便清晰看到）
            const axesHelper = new THREE.AxesHelper(0.08);
            axesHelper.renderOrder = 999;
            // 设置坐标轴线条更粗一些
            axesHelper.material.linewidth = 2;
            if (axesHelper.material) {
                axesHelper.material.depthTest = false;
                axesHelper.material.depthWrite = false;
                axesHelper.material.transparent = true;
                axesHelper.material.opacity = 0.95;
            }
            // 确保坐标轴scale为(1,1,1)
            axesHelper.scale.set(1, 1, 1);
            this.absolutePoseMarker.add(axesHelper);
            
            this.absolutePoseMarker.renderOrder = 998;
            this.scene.add(this.absolutePoseMarker);
            
            console.log('  ✓ 绝对位姿标记已创建（带坐标轴）');
        }
        
        // 相对位姿连线（从robot1到robot2）
        if (visConfig.show_relative_pose !== false) {
            let color = visConfig.relative_pose_color || 0xef4444;
            if (typeof color === 'string' && color.startsWith('#')) {
                color = parseInt(color.substring(1), 16);
            }
            const geometry = new THREE.BufferGeometry();
            const positions = new Float32Array(6); // 两个点，每个3个坐标
            geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            const material = new THREE.LineBasicMaterial({ 
                color: color,
                linewidth: 2,
                transparent: true,
                opacity: 0.6
            });
            this.relativePoseLine = new THREE.Line(geometry, material);
            this.relativePoseLine.renderOrder = 997;
            this.scene.add(this.relativePoseLine);
        }
    }
    
    /**
     * 更新机器人位姿（支持单臂和双臂模式）
     */
    updateRobotPose(jointAngles, pose, jointNamesOrder, robotIndex = 0) {
        if (this.dualArmMode) {
            // 双臂模式：robotIndex 指定更新哪个机器人 (0 或 1)
            this.updateDualArmPose(jointAngles, pose, jointNamesOrder, robotIndex);
        } else {
            // 单臂模式：保持原有逻辑
            this.updateSingleArmPose(jointAngles, pose, jointNamesOrder);
        }
    }
    
    /**
     * 更新单臂机器人位姿
     */
    updateSingleArmPose(jointAngles, pose, jointNamesOrder) {
        if (!pose || !pose.translation || !pose.rotation) {
            return;
        }
        
        try {
            // 获取 DQ 位置 (索引 1,2,3 = x,y,z)
            // DQ translation 返回格式: [w, x, y, z]，其中 w=0（纯平移）
            // 注意：DQ Robotics 使用的是 Z-up 坐标系，与 Three.js 一致
            let x = pose.translation[1] || 0;
            let y = pose.translation[2] || 0;
            let z = pose.translation[3] || 0;
            
            // 调试：打印原始坐标（仅在开发模式下）
            // 注意：DQ Robotics 的 fkm() 返回的坐标是相对于 reference_frame 的
            // 如果设置了 set_reference_frame(base)，那么返回的坐标是相对于基座坐标系的
            // 如果基座有旋转，这里的坐标需要变换到世界坐标系
            // 但是，由于我们同时设置了 set_base_frame 和 set_reference_frame 为同一个值，
            // fkm() 应该返回相对于世界坐标系的坐标（因为 reference_frame 已经包含了基座变换）
            
            // 更新末端执行器位置
            // 使用DQ Robotics通过MDH参数计算的正向运动学结果
            if (this.endEffector) {
                // 使用正向运动学计算的位置（已经包含了末端偏移，如果设置了的话）
                // DQ Robotics 返回的坐标格式：[w, x, y, z]，其中 w=0（纯平移）
                // 索引 1,2,3 分别对应 x, y, z
                this.endEffector.position.set(x, y, z);
                
                // 更新旋转
                const qw = pose.rotation[0] || 1;
                const qx = pose.rotation[1] || 0;
                const qy = pose.rotation[2] || 0;
                const qz = pose.rotation[3] || 0;
                
                this.endEffector.quaternion.set(qx, qy, qz, qw);
            }
            
            // 更新 URDF 关节
            if (this.urdfRobot && jointAngles) {
                this.updateURDFJoints(this.urdfRobot, jointAngles, jointNamesOrder);
            }
            
            // 更新拖动控制位置（确保一直跟随末端执行器）
            if (this.dragEndEffectorMode) {
                this.updateGizmoPosition();
            }
            
        } catch (error) {
            console.error('更新单臂位姿失败:', error);
        }
    }
    
    /**
     * 更新双臂机器人位姿
     */
    updateDualArmPose(jointAngles, pose, jointNamesOrder, robotIndex) {
        if (!pose || !pose.translation || !pose.rotation) {
            return;
        }
        
        try {
            const x = pose.translation[1] || 0;
            const y = pose.translation[2] || 0;
            const z = pose.translation[3] || 0;
            const qw = pose.rotation[0] || 1;
            const qx = pose.rotation[1] || 0;
            const qy = pose.rotation[2] || 0;
            const qz = pose.rotation[3] || 0;
            
            if (robotIndex === 0) {
                // 更新机器人1
                if (this.robot1EndEffector) {
                    this.robot1EndEffector.position.set(x, y, z);
                    this.robot1EndEffector.quaternion.set(qx, qy, qz, qw);
                }
                if (this.robot1URDF && jointAngles) {
                    this.updateURDFJoints(this.robot1URDF, jointAngles, jointNamesOrder);
                }
            } else if (robotIndex === 1) {
                // 更新机器人2
                if (this.robot2EndEffector) {
                    this.robot2EndEffector.position.set(x, y, z);
                    this.robot2EndEffector.quaternion.set(qx, qy, qz, qw);
                }
                if (this.robot2URDF && jointAngles) {
                    this.updateURDFJoints(this.robot2URDF, jointAngles, jointNamesOrder);
                }
            }
        } catch (error) {
            console.error(`更新机器人${robotIndex + 1}位姿失败:`, error);
        }
    }
    
    /**
     * 更新双臂绝对位姿和相对位姿可视化
     */
    updateDualArmRelativeVisualizations(pose1, pose2, absolutePose = null, relativePose = null) {
        if (!this.dualArmMode) return;
        
        try {
            // 更新绝对位姿标记（两个末端执行器的中点）
            if (this.absolutePoseMarker && pose1 && pose2) {
                const x1 = pose1.translation[1] || 0;
                const y1 = pose1.translation[2] || 0;
                const z1 = pose1.translation[3] || 0;
                const x2 = pose2.translation[1] || 0;
                const y2 = pose2.translation[2] || 0;
                const z2 = pose2.translation[3] || 0;
                
                const midX = (x1 + x2) / 2;
                const midY = (y1 + y2) / 2;
                const midZ = (z1 + z2) / 2;
                
                // 更新位置
                this.absolutePoseMarker.position.set(midX, midY, midZ);
                
                // 确保scale始终为(1,1,1)，避免变形为椭球
                this.absolutePoseMarker.scale.set(1, 1, 1);
                
                // 更新旋转（如果有计算好的绝对位姿四元数）
                if (absolutePose && absolutePose.rotation) {
                    const qw = absolutePose.rotation[0] || 1;
                    const qx = absolutePose.rotation[1] || 0;
                    const qy = absolutePose.rotation[2] || 0;
                    const qz = absolutePose.rotation[3] || 0;
                    
                    // 归一化四元数，确保旋转正确
                    const len = Math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
                    if (len > 0.001) {
                        this.absolutePoseMarker.quaternion.set(
                            qx / len, 
                            qy / len, 
                            qz / len, 
                            qw / len
                        );
                    } else {
                        // 如果四元数无效，使用单位四元数
                        this.absolutePoseMarker.quaternion.set(0, 0, 0, 1);
                    }
                } else {
                    // 如果没有提供绝对位姿旋转，使用两个末端的平均旋转
                    const qw1 = pose1.rotation[0] || 1;
                    const qx1 = pose1.rotation[1] || 0;
                    const qy1 = pose1.rotation[2] || 0;
                    const qz1 = pose1.rotation[3] || 0;
                    const qw2 = pose2.rotation[0] || 1;
                    const qx2 = pose2.rotation[1] || 0;
                    const qy2 = pose2.rotation[2] || 0;
                    const qz2 = pose2.rotation[3] || 0;
                    
                    // 平均四元数（简单平均）
                    let avgQW = (qw1 + qw2) / 2;
                    let avgQX = (qx1 + qx2) / 2;
                    let avgQY = (qy1 + qy2) / 2;
                    let avgQZ = (qz1 + qz2) / 2;
                    
                    // 归一化
                    const len = Math.sqrt(avgQW * avgQW + avgQX * avgQX + avgQY * avgQY + avgQZ * avgQZ);
                    if (len > 0.001) {
                        this.absolutePoseMarker.quaternion.set(
                            avgQX / len, 
                            avgQY / len, 
                            avgQZ / len, 
                            avgQW / len
                        );
                    } else {
                        // 如果平均后接近零，使用默认单位四元数
                        this.absolutePoseMarker.quaternion.set(0, 0, 0, 1);
                    }
                }
            }
            
            // 更新相对位姿连线
            if (this.relativePoseLine && pose1 && pose2) {
                const x1 = pose1.translation[1] || 0;
                const y1 = pose1.translation[2] || 0;
                const z1 = pose1.translation[3] || 0;
                const x2 = pose2.translation[1] || 0;
                const y2 = pose2.translation[2] || 0;
                const z2 = pose2.translation[3] || 0;
                
                const positions = this.relativePoseLine.geometry.attributes.position;
                positions.array[0] = x1;
                positions.array[1] = y1;
                positions.array[2] = z1;
                positions.array[3] = x2;
                positions.array[4] = y2;
                positions.array[5] = z2;
                positions.needsUpdate = true;
            }
        } catch (error) {
            console.error('更新双臂可视化失败:', error);
        }
    }
    
    /**
     * 更新 URDF 关节角度（通用方法）
     */
    updateURDFJoints(urdfRobot, jointAngles, jointNamesOrder) {
        if (!urdfRobot || !jointAngles) return;
        
        if (Array.isArray(jointNamesOrder) && jointNamesOrder.length > 0) {
            jointNamesOrder.forEach((name, i) => {
                const j = urdfRobot.joints[name];
                if (j && (j.jointType === 'revolute' || j.jointType === 'continuous' || j.jointType === 'prismatic')) {
                    j.setJointValue(jointAngles[i] || 0);
                }
            });
        } else {
            // 回退：自动查找所有旋转关节
            const allJoints = Object.keys(urdfRobot.joints);
            const revoluteJoints = allJoints.filter(name => {
                const joint = urdfRobot.joints[name];
                return joint.jointType === 'revolute' || joint.jointType === 'continuous';
            });
            revoluteJoints.forEach((name, i) => {
                if (i < jointAngles.length && urdfRobot.joints[name]) {
                    urdfRobot.joints[name].setJointValue(jointAngles[i]);
                }
            });
        }
    }
    
    /**
     * 显示/隐藏MDH坐标系
     * @param {Array} mdhFrames - MDH坐标系数组，每个元素包含 {origin, xAxis, yAxis, zAxis}
     */
    setMDHFrames(mdhFrames) {
        console.log('setMDHFrames 被调用，mdhFrames数量:', mdhFrames ? mdhFrames.length : 0);
        console.log('当前 showMDHFrames 状态:', this.showMDHFrames);
        
        // 先移除旧的MDH坐标系
        if (this.mdhFramesGroup) {
            this.scene.remove(this.mdhFramesGroup);
            this.mdhFramesGroup = null;
        }
        
        if (!mdhFrames || mdhFrames.length === 0) {
            console.warn('MDH坐标系数据为空，无法创建');
            return;
        }
        
        // 存储最后一个MDH坐标系的位置，用于更新末端执行器
        if (mdhFrames.length > 0) {
            const lastFrame = mdhFrames[mdhFrames.length - 1];
            this.lastMDHFrameOrigin = lastFrame.origin.clone();
            console.log(`存储最后一个MDH坐标系位置: (${this.lastMDHFrameOrigin.x.toFixed(4)}, ${this.lastMDHFrameOrigin.y.toFixed(4)}, ${this.lastMDHFrameOrigin.z.toFixed(4)})`);
        }
        
        // 创建MDH坐标系组
        this.mdhFramesGroup = new THREE.Group();
        this.mdhFramesGroup.name = 'MDHFrames';
        this.mdhFramesGroup.renderOrder = 1000; // 设置高渲染优先级，确保在机器人之前渲染
        
        const axisLength = 0.1; // 正常大小的坐标轴长度
        
        mdhFrames.forEach((frame, index) => {
            const { origin, xAxis, yAxis, zAxis } = frame;
            
            // 创建坐标系组
            const frameGroup = new THREE.Group();
            frameGroup.position.copy(origin);
            frameGroup.renderOrder = 1000; // 高渲染优先级
            
            // 创建旋转矩阵（从X、Y、Z轴构建）
            const rotationMatrix = new THREE.Matrix4();
            rotationMatrix.makeBasis(xAxis, yAxis, zAxis);
            frameGroup.setRotationFromMatrix(rotationMatrix);
            
            // 使用AxesHelper创建坐标轴（正常粗细）
            const axesHelper = new THREE.AxesHelper(axisLength);
            // 设置线条材质，禁用深度测试以确保始终显示在前面
            axesHelper.material.linewidth = 2;
            axesHelper.material.depthTest = false; // 禁用深度测试，始终显示在前面
            axesHelper.material.depthWrite = false;
            axesHelper.renderOrder = 1000; // 确保在机器人之前渲染
            frameGroup.add(axesHelper);
            
            // 添加标签（使用Sprite）
            const createAxisLabel = (text, color, offset) => {
                const canvas = document.createElement('canvas');
                const context = canvas.getContext('2d');
                canvas.width = 64;
                canvas.height = 64;
                
                context.fillStyle = color;
                context.font = 'Bold 48px Arial';
                context.textAlign = 'center';
                context.textBaseline = 'middle';
                context.fillText(text, 32, 32);
                
                const texture = new THREE.CanvasTexture(canvas);
                const spriteMaterial = new THREE.SpriteMaterial({ 
                    map: texture,
                    depthTest: false, // 禁用深度测试，始终显示在前面
                    depthWrite: false,
                    transparent: true
                });
                const sprite = new THREE.Sprite(spriteMaterial);
                sprite.position.copy(offset);
                sprite.scale.set(0.02, 0.02, 1);
                sprite.renderOrder = 1001; // 设置更高的渲染顺序，确保在最前面
                frameGroup.add(sprite);
            };
            
            createAxisLabel('X', '#ef4444', new THREE.Vector3(0, axisLength + 0.02, 0));
            createAxisLabel('Y', '#10b981', new THREE.Vector3(-axisLength - 0.02, 0, 0));
            createAxisLabel('Z', '#3b82f6', new THREE.Vector3(0, 0, axisLength + 0.02));
            
            // 添加坐标系索引标签
            const createFrameLabel = (text) => {
                const canvas = document.createElement('canvas');
                const context = canvas.getContext('2d');
                // 增大canvas尺寸，确保文本完全显示
                canvas.width = 512;
                canvas.height = 256;
                
                // 清除背景（透明）
                context.clearRect(0, 0, canvas.width, canvas.height);
                
                context.fillStyle = '#ffffff';
                context.font = 'Bold 120px Arial'; // 增大字体
                context.textAlign = 'center';
                context.textBaseline = 'middle';
                // 在canvas中心绘制文本
                context.fillText(text, canvas.width / 2, canvas.height / 2);
                
                const texture = new THREE.CanvasTexture(canvas);
                texture.needsUpdate = true;
                const spriteMaterial = new THREE.SpriteMaterial({ 
                    map: texture,
                    depthTest: false, // 禁用深度测试，始终显示在前面
                    depthWrite: false,
                    transparent: true
                });
                const sprite = new THREE.Sprite(spriteMaterial);
                // Frame标签位置：在Y轴负方向（在坐标系本地空间中）
                sprite.position.set(0, -axisLength - 0.05, 0);
                sprite.scale.set(0.08, 0.04, 1); // 相应增大sprite尺寸
                sprite.renderOrder = 1001; // 设置更高的渲染顺序，确保在最前面
                frameGroup.add(sprite);
            };
            
            createFrameLabel(`Frame ${index}`);
            
            this.mdhFramesGroup.add(frameGroup);
        });
        
        // 根据显示状态决定是否添加到场景
        // 确保MDH坐标系在机器人之前添加到场景（渲染顺序）
        if (this.showMDHFrames) {
            console.log('showMDHFrames 为 true，将MDH坐标系添加到场景');
            // 如果场景中已有MDH坐标系组，先移除
            const existingFrames = this.scene.getObjectByName('MDHFrames');
            if (existingFrames) {
                this.scene.remove(existingFrames);
            }
            // 将MDH坐标系添加到场景
            this.scene.add(this.mdhFramesGroup);
            console.log('MDH坐标系已添加到场景，场景对象数:', this.scene.children.length);
        } else {
            console.log('showMDHFrames 为 false，MDH坐标系已创建但未添加到场景');
            console.log('提示：请打开"显示MDH坐标系"开关来显示坐标系');
        }
        
        console.log(`✓ 已创建 ${mdhFrames.length} 个MDH坐标系`);
    }
    
    /**
     * 切换MDH坐标系显示
     * @param {boolean} show - 是否显示
     */
    toggleMDHFrames(show) {
        this.showMDHFrames = show !== undefined ? show : !this.showMDHFrames;
        console.log('toggleMDHFrames 被调用，新状态:', this.showMDHFrames);
        
        if (this.mdhFramesGroup) {
            if (this.showMDHFrames) {
                // 检查是否已经在场景中
                if (!this.scene.getObjectByName('MDHFrames')) {
                    this.scene.add(this.mdhFramesGroup);
                    console.log('MDH坐标系已添加到场景');
                } else {
                    console.log('MDH坐标系已在场景中');
                }
            } else {
                this.scene.remove(this.mdhFramesGroup);
                console.log('MDH坐标系已从场景移除');
            }
        } else {
            console.warn('MDH坐标系组不存在，无法切换显示');
            console.warn('提示：请先提取MDH参数以创建坐标系');
        }
    }
    
    /**
     * 设置拖动末端执行器模式
     * @param {boolean} enabled - 是否启用拖动模式
     */
    setDragEndEffectorMode(enabled) {
        this.dragEndEffectorMode = enabled;
        if (enabled) {
            this.createEndEffectorGizmo();
        } else {
            this.removeEndEffectorGizmo();
        }
        console.log(`拖动末端执行器模式: ${enabled ? '启用' : '禁用'}`);
    }
    
    /**
     * 创建末端执行器拖动控制（MoveIt 风格）
     */
    createEndEffectorGizmo() {
        if (this.endEffectorGizmo) {
            this.scene.remove(this.endEffectorGizmo);
        }
        
        this.endEffectorGizmo = new THREE.Group();
        this.endEffectorGizmo.name = 'EndEffectorGizmo';
        
        const arrowLength = 0.3;   // 增大箭头长度
        const arrowRadius = 0.04;  // 增大箭头半径（更粗，更容易点击）
        const coneLength = 0.1;    // 增大箭头头部长度
        const coneRadius = 0.08;   // 增大箭头头部半径（更粗）
        const arrowOffset = 0.1;   // 箭头之间的间隔（从中心点偏移，增大间隔避免重叠）
        
        // 创建平移箭头（X, Y, Z 轴）
        const colors = {
            x: 0xff0000,  // 红色 - X轴
            y: 0x00ff00,  // 绿色 - Y轴
            z: 0x0000ff   // 蓝色 - Z轴
        };
        
        const axes = [
            { name: 'x', dir: new THREE.Vector3(1, 0, 0) },
            { name: 'y', dir: new THREE.Vector3(0, 1, 0) },
            { name: 'z', dir: new THREE.Vector3(0, 0, 1) }
        ];
        
        axes.forEach((axis) => {
            // 创建正方向箭头（从中心点向正方向偏移）
            const positiveStart = axis.dir.clone().multiplyScalar(arrowOffset);
            const arrowHelperPositive = new THREE.ArrowHelper(
                axis.dir,
                positiveStart,  // 起始位置偏移，避免重叠
                arrowLength,
                colors[axis.name],
                coneLength,
                arrowRadius
            );
            
            // 设置箭头材质（确保显示在最前端）
            arrowHelperPositive.line.material.transparent = true;
            arrowHelperPositive.line.material.opacity = 0.8;
            arrowHelperPositive.line.material.depthTest = false;  // 不进行深度测试，始终显示在前面
            arrowHelperPositive.line.material.depthWrite = false;  // 不写入深度缓冲
            arrowHelperPositive.line.renderOrder = 1000;  // 高渲染优先级
            
            arrowHelperPositive.cone.material.transparent = true;
            arrowHelperPositive.cone.material.opacity = 0.8;
            arrowHelperPositive.cone.material.depthTest = false;  // 不进行深度测试
            arrowHelperPositive.cone.material.depthWrite = false;  // 不写入深度缓冲
            arrowHelperPositive.cone.renderOrder = 1000;  // 高渲染优先级
            
            // 设置整个箭头组的渲染顺序
            arrowHelperPositive.renderOrder = 1000;
            
            arrowHelperPositive.userData = { 
                type: 'translation', 
                axis: axis.name, 
                direction: 'positive' 
            };
            
            // 创建负方向箭头（从中心点向负方向偏移）
            const negativeStart = axis.dir.clone().negate().multiplyScalar(arrowOffset);
            const arrowHelperNegative = new THREE.ArrowHelper(
                axis.dir.clone().negate(),  // 反向
                negativeStart,  // 起始位置偏移，避免重叠
                arrowLength,
                colors[axis.name],
                coneLength,
                arrowRadius
            );
            
            // 设置箭头材质（确保显示在最前端）
            arrowHelperNegative.line.material.transparent = true;
            arrowHelperNegative.line.material.opacity = 0.8;
            arrowHelperNegative.line.material.depthTest = false;  // 不进行深度测试，始终显示在前面
            arrowHelperNegative.line.material.depthWrite = false;  // 不写入深度缓冲
            arrowHelperNegative.line.renderOrder = 1000;  // 高渲染优先级
            
            arrowHelperNegative.cone.material.transparent = true;
            arrowHelperNegative.cone.material.opacity = 0.8;
            arrowHelperNegative.cone.material.depthTest = false;  // 不进行深度测试
            arrowHelperNegative.cone.material.depthWrite = false;  // 不写入深度缓冲
            arrowHelperNegative.cone.renderOrder = 1000;  // 高渲染优先级
            
            // 设置整个箭头组的渲染顺序
            arrowHelperNegative.renderOrder = 1000;
            
            arrowHelperNegative.userData = { 
                type: 'translation', 
                axis: axis.name, 
                direction: 'negative' 
            };
            
            // 存储箭头（使用对象存储正负两个方向）
            this.translationArrows[axis.name] = {
                positive: arrowHelperPositive,
                negative: arrowHelperNegative
            };
            
            this.endEffectorGizmo.add(arrowHelperPositive);
            this.endEffectorGizmo.add(arrowHelperNegative);
        });
        
        // 暂时禁用旋转箭头功能（只保留平移功能）
        // 创建旋转箭头（围绕 X, Y, Z 轴）
        // const rotationRadius = 0.12;
        // const rotationThickness = 0.008;
        // 
        // axes.forEach((axis) => {
        //     // 创建圆环（旋转箭头）
        //     const torusGeometry = new THREE.TorusGeometry(rotationRadius, rotationThickness, 16, 64);
        //     const torusMaterial = new THREE.MeshBasicMaterial({ 
        //         color: colors[axis.name],
        //         transparent: true,
        //         opacity: 0.6,
        //         side: THREE.DoubleSide
        //     });
        //     const torus = new THREE.Mesh(torusGeometry, torusMaterial);
        //     
        //     // 根据轴方向旋转圆环
        //     if (axis.name === 'x') {
        //         torus.rotateZ(Math.PI / 2);
        //     } else if (axis.name === 'y') {
        //         torus.rotateX(Math.PI / 2);
        //     }
        //     // z轴不需要旋转
        //     
        //     torus.userData = { type: 'rotation', axis: axis.name };
        //     
        //     this.rotationArrows[axis.name] = torus;
        //     this.endEffectorGizmo.add(torus);
        // });
        
        // 确保整个 gizmo 组显示在最前端
        this.endEffectorGizmo.renderOrder = 1000;
        
        // 遍历所有子对象，确保都设置正确的渲染顺序和深度测试
        this.endEffectorGizmo.traverse((child) => {
            if (child instanceof THREE.Mesh || child instanceof THREE.Line) {
                if (child.material) {
                    child.material.depthTest = false;
                    child.material.depthWrite = false;
                }
                child.renderOrder = 1000;
            }
        });
        
        this.scene.add(this.endEffectorGizmo);
        
        // 更新位置到末端执行器
        this.updateGizmoPosition();
    }
    
    /**
     * 移除拖动控制
     */
    removeEndEffectorGizmo() {
        if (this.endEffectorGizmo) {
            this.scene.remove(this.endEffectorGizmo);
            this.endEffectorGizmo = null;
            this.translationArrows = { x: null, y: null, z: null };
            this.rotationArrows = { x: null, y: null, z: null };
        }
    }
    
    /**
     * 更新拖动控制的位置（跟随末端执行器）
     */
    updateGizmoPosition() {
        if (this.endEffectorGizmo && this.endEffector) {
            this.endEffectorGizmo.position.copy(this.endEffector.position);
            this.endEffectorGizmo.quaternion.copy(this.endEffector.quaternion);
        }
    }
    
    /**
     * 处理拖动末端执行器的鼠标按下事件
     */
    handleDragEndEffectorMouseDown(e) {
        if (!this.dragEndEffectorMode || this.dualArmMode || !this.endEffectorGizmo) return;
        if (e.button !== 0) return; // 只处理左键
        
        // 更新鼠标位置
        this.updateMousePosition(e);
        
        // 检测是否点击在箭头上（只检测平移箭头）
        // 收集所有箭头（包括正负方向）
        const allArrows = [];
        Object.values(this.translationArrows).forEach(arrowPair => {
            if (arrowPair && arrowPair.positive) {
                allArrows.push(arrowPair.positive);
            }
            if (arrowPair && arrowPair.negative) {
                allArrows.push(arrowPair.negative);
            }
        });
        
        const intersects = [];
        allArrows.forEach(arrow => {
            const arrowIntersects = this.raycaster.intersectObject(arrow, true);
            arrowIntersects.forEach(intersect => {
                intersect.object = arrow;
                intersects.push(intersect);
            });
        });
        
        if (intersects.length > 0) {
            // 找到最近的交点
            intersects.sort((a, b) => a.distance - b.distance);
            const closest = intersects[0];
            
            // 获取箭头信息
            const arrowObject = closest.object;
            let arrowGroup = arrowObject;
            
            // 如果是箭头的一部分（ArrowHelper 的 line 或 cone），找到父组
            while (arrowGroup && arrowGroup.parent && arrowGroup.parent !== this.endEffectorGizmo) {
                arrowGroup = arrowGroup.parent;
            }
            
            // 如果还是没有找到，检查是否是 ArrowHelper 本身或其父对象
            if (!arrowGroup || !arrowGroup.userData || !arrowGroup.userData.type) {
                // 检查是否是 ArrowHelper 的子对象
                if (arrowObject.parent && arrowObject.parent.userData && arrowObject.parent.userData.type) {
                    arrowGroup = arrowObject.parent;
                } else if (arrowObject instanceof THREE.ArrowHelper && arrowObject.userData && arrowObject.userData.type) {
                    arrowGroup = arrowObject;
                }
            }
            
            if (arrowGroup && arrowGroup.userData && arrowGroup.userData.type) {
                this.selectedArrow = arrowGroup;
                this.arrowType = arrowGroup.userData.type;
                this.arrowAxis = arrowGroup.userData.axis;
                this.isDraggingEndEffector = true;
                
                // 存储箭头方向信息，确保在拖动过程中也能访问
                this.selectedArrowDirection = arrowGroup.userData.direction || 'positive';
                
                // 高亮选中的箭头
                this.highlightArrow(arrowGroup, true);
                
                // 创建拖动平面（根据箭头类型）
                if (this.arrowType === 'translation') {
                    // 平移：平面垂直于箭头方向
                    const axisDir = this.getAxisDirection(this.arrowAxis);
                    // 根据方向（正/负）决定箭头方向
                    const arrowDirection = this.selectedArrowDirection === 'negative' ? -1 : 1;
                    const worldAxisDir = axisDir.clone().multiplyScalar(arrowDirection).applyQuaternion(this.endEffectorGizmo.quaternion);
                    
                    // 【锁定方向】存储世界轴方向，供拖动时使用（在整个拖动过程中保持不变）
                    this.worldAxisDirection = worldAxisDir.clone().normalize();
                    
                    // 【锁定拖动平面】创建拖动平面：平面应该垂直于箭头方向
                    // 平面的法向量应该垂直于箭头方向，这样鼠标在平面上的移动可以投影到箭头方向
                    // 使用相机方向作为平面的法向量（如果相机方向与箭头方向平行，使用相机到末端执行器的方向）
                    const cameraDirection = new THREE.Vector3();
                    this.camera.getWorldDirection(cameraDirection);
                    
                    // 计算垂直于箭头方向且接近相机方向的平面法向量
                    let planeNormal = cameraDirection.clone();
                    
                    // 如果相机方向与箭头方向平行（点积接近1或-1），使用相机到末端执行器的方向
                    const dotProduct = Math.abs(planeNormal.dot(this.worldAxisDirection));
                    if (dotProduct > 0.9) {
                        // 相机方向与箭头方向太接近平行，使用相机到末端执行器的方向
                        const cameraToEndEffector = new THREE.Vector3().subVectors(this.endEffector.position, this.camera.position).normalize();
                        planeNormal = cameraToEndEffector;
                    }
                    
                    // 确保平面法向量与箭头方向垂直（通过从相机方向减去箭头方向的投影）
                    const projection = planeNormal.dot(this.worldAxisDirection);
                    planeNormal.sub(this.worldAxisDirection.clone().multiplyScalar(projection)).normalize();
                    
                    // 如果平面法向量为零（说明相机方向与箭头方向完全平行），使用一个默认的垂直方向
                    if (planeNormal.length() < 0.1) {
                        // 选择一个与箭头方向垂直的默认方向
                        if (Math.abs(this.worldAxisDirection.x) < 0.9) {
                            planeNormal.set(1, 0, 0).sub(this.worldAxisDirection.clone().multiplyScalar(this.worldAxisDirection.x)).normalize();
                        } else {
                            planeNormal.set(0, 1, 0).sub(this.worldAxisDirection.clone().multiplyScalar(this.worldAxisDirection.y)).normalize();
                        }
                    }
                    
                    // 【锁定平面法向量】存储平面法向量，确保在整个拖动过程中保持不变
                    this.lockedPlaneNormal = planeNormal.clone();
                    
                    // 调试信息
                    console.log(`✓ 选中箭头: ${this.arrowAxis} ${this.selectedArrowDirection}`);
                    console.log(`  局部轴方向: (${axisDir.x}, ${axisDir.y}, ${axisDir.z})`);
                    console.log(`  箭头方向系数: ${arrowDirection}`);
                    console.log(`  箭头方向（世界坐标系）: (${worldAxisDir.x.toFixed(4)}, ${worldAxisDir.y.toFixed(4)}, ${worldAxisDir.z.toFixed(4)})`);
                    console.log(`  平面法向量（垂直于箭头）: (${planeNormal.x.toFixed(4)}, ${planeNormal.y.toFixed(4)}, ${planeNormal.z.toFixed(4)})`);
                    console.log(`  验证垂直性: ${planeNormal.dot(worldAxisDir).toFixed(6)} (应该接近0)`);
                    
                    // 【锁定拖动平面】创建平面：法向量垂直于箭头方向，经过末端执行器位置
                    // Plane(normal, constant) 其中 constant = -normal.dot(point)
                    // 使用锁定的平面法向量，确保在整个拖动过程中保持不变
                    const planeConstant = -this.lockedPlaneNormal.dot(this.endEffector.position);
                    this.dragPlane = new THREE.Plane(this.lockedPlaneNormal, planeConstant);
                    
                    // 初始化拖动起始位置（在拖动平面上）
                    const initialIntersection = new THREE.Vector3();
                    this.updateMousePosition(e);
                    this.raycaster.ray.intersectPlane(this.dragPlane, initialIntersection);
                    
                    this.dragStartPosition = initialIntersection.clone();
                    this.dragCurrentPosition = initialIntersection.clone();
                    this.previousMousePosition = { x: e.clientX, y: e.clientY };
                    
                    console.log(`  拖动起始位置（平面交点）: (${initialIntersection.x.toFixed(4)}, ${initialIntersection.y.toFixed(4)}, ${initialIntersection.z.toFixed(4)})`);
                } else {
                    // 旋转：平面垂直于相机视线
                    const cameraDirection = new THREE.Vector3();
                    this.camera.getWorldDirection(cameraDirection);
                    this.dragPlane = new THREE.Plane(cameraDirection, -cameraDirection.dot(this.endEffector.position));
                    
                    this.dragStartPosition = this.endEffector.position.clone();
                    this.dragCurrentPosition = this.endEffector.position.clone();
                    this.previousMousePosition = { x: e.clientX, y: e.clientY };
                }
                
                e.preventDefault();
                e.stopPropagation();
            }
        }
    }
    
    /**
     * 获取轴方向向量
     */
    getAxisDirection(axis) {
        const directions = {
            x: new THREE.Vector3(1, 0, 0),
            y: new THREE.Vector3(0, 1, 0),
            z: new THREE.Vector3(0, 0, 1)
        };
        return directions[axis] || new THREE.Vector3(0, 0, 1);
    }
    
    /**
     * 高亮箭头
     */
    highlightArrow(arrow, highlight) {
        if (!arrow) return;
        
        // 如果是 ArrowHelper，特殊处理
        if (arrow instanceof THREE.ArrowHelper) {
            // 处理 line（LineBasicMaterial）
            if (arrow.line && arrow.line.material) {
                if (!arrow.line.userData.originalColor) {
                    arrow.line.userData.originalColor = arrow.line.material.color.clone();
                    arrow.line.userData.originalOpacity = arrow.line.material.opacity;
                }
                if (highlight) {
                    arrow.line.material.color.setHex(0xffffff);
                    arrow.line.material.opacity = 1.0;
                } else {
                    arrow.line.material.color.copy(arrow.line.userData.originalColor);
                    arrow.line.material.opacity = arrow.line.userData.originalOpacity || 0.8;
                }
            }
            
            // 处理 cone（MeshBasicMaterial）
            if (arrow.cone && arrow.cone.material) {
                if (!arrow.cone.userData.originalOpacity) {
                    arrow.cone.userData.originalOpacity = arrow.cone.material.opacity;
                }
                if (arrow.cone.material.emissive) {
                    if (highlight) {
                        arrow.cone.material.emissive.setHex(0xffffff);
                        if (arrow.cone.material.emissiveIntensity !== undefined) {
                            arrow.cone.material.emissiveIntensity = 0.5;
                        }
                        arrow.cone.material.opacity = 1.0;
                        if (arrow.cone.scale) {
                            arrow.cone.scale.set(1.3, 1.3, 1.3);
                        }
                    } else {
                        arrow.cone.material.emissive.setHex(0x000000);
                        if (arrow.cone.material.emissiveIntensity !== undefined) {
                            arrow.cone.material.emissiveIntensity = 0;
                        }
                        arrow.cone.material.opacity = arrow.cone.userData.originalOpacity || 0.8;
                        if (arrow.cone.scale) {
                            arrow.cone.scale.set(1, 1, 1);
                        }
                    }
                } else {
                    // 如果没有 emissive，使用 color
                    if (!arrow.cone.userData.originalColor) {
                        arrow.cone.userData.originalColor = arrow.cone.material.color.clone();
                    }
                    if (highlight) {
                        arrow.cone.material.color.setHex(0xffffff);
                        arrow.cone.material.opacity = 1.0;
                        if (arrow.cone.scale) {
                            arrow.cone.scale.set(1.3, 1.3, 1.3);
                        }
                    } else {
                        arrow.cone.material.color.copy(arrow.cone.userData.originalColor);
                        arrow.cone.material.opacity = arrow.cone.userData.originalOpacity || 0.8;
                        if (arrow.cone.scale) {
                            arrow.cone.scale.set(1, 1, 1);
                        }
                    }
                }
            }
            return;
        }
        
        // 处理其他类型的箭头（Mesh）
        arrow.traverse((child) => {
            if (child instanceof THREE.Mesh && child.material) {
                if (!child.userData.originalOpacity) {
                    child.userData.originalOpacity = child.material.opacity;
                    if (child.material.color) {
                        child.userData.originalColor = child.material.color.clone();
                    }
                }
                
                if (highlight) {
                    // 优先使用 emissive，如果没有则使用 color
                    if (child.material.emissive) {
                        child.material.emissive.setHex(0xffffff);
                        if (child.material.emissiveIntensity !== undefined) {
                            child.material.emissiveIntensity = 0.5;
                        }
                    } else if (child.material.color) {
                        child.material.color.setHex(0xffffff);
                    }
                    child.material.opacity = 1.0;
                    if (child.scale) {
                        child.scale.set(1.3, 1.3, 1.3);
                    }
                } else {
                    // 恢复原始状态
                    if (child.material.emissive) {
                        child.material.emissive.setHex(0x000000);
                        if (child.material.emissiveIntensity !== undefined) {
                            child.material.emissiveIntensity = 0;
                        }
                    } else if (child.material.color && child.userData.originalColor) {
                        child.material.color.copy(child.userData.originalColor);
                    }
                    child.material.opacity = child.userData.originalOpacity || 0.8;
                    if (child.scale) {
                        child.scale.set(1, 1, 1);
                    }
                }
            }
        });
    }
    
    /**
     * 处理拖动末端执行器的鼠标移动事件
     */
    handleDragEndEffectorMouseMove(e) {
        if (!this.dragEndEffectorMode || !this.isDraggingEndEffector || this.dualArmMode || !this.endEffector || !this.selectedArrow || !this.dragPlane) return;
        
        // 更新鼠标位置
        this.updateMousePosition(e);
        
        if (this.arrowType === 'translation') {
            // 平移箭头拖动
            const intersection = new THREE.Vector3();
            const hasIntersection = this.raycaster.ray.intersectPlane(this.dragPlane, intersection);
            
            if (hasIntersection) {
                // 【使用锁定的方向】计算沿箭头方向的位移
                // 使用存储的世界轴方向（在鼠标按下时已经锁定，在整个拖动过程中保持不变）
                // 确保方向完全锁定，不重新计算
                if (!this.worldAxisDirection) {
                    console.error('拖动方向未锁定！');
                    return;
                }
                const worldAxisDir = this.worldAxisDirection.clone();
                
                // 计算鼠标在拖动平面上的移动（从上次位置到当前位置）
                // 注意：这里需要先克隆，避免修改原始向量
                const deltaPos = intersection.clone().sub(this.dragCurrentPosition);
                
                // 计算鼠标移动在箭头方向上的投影长度
                // 这个投影长度就是沿箭头方向的位移
                const projectionLength = deltaPos.dot(worldAxisDir);
                
                // 计算沿箭头方向的位移向量
                // projectedDelta 的方向就是 worldAxisDir 的方向
                // 如果投影长度为正，说明沿箭头正方向移动；为负则沿负方向移动
                const projectedDelta = worldAxisDir.clone().multiplyScalar(projectionLength);
                
                // 调试信息（限制频率）
                if (!this._debugLogged || (Date.now() - (this._lastDebugTime || 0)) > 500) {
                    console.log(`拖动: 箭头=${this.arrowAxis} ${this.selectedArrowDirection}`);
                    console.log(`  交点: (${intersection.x.toFixed(4)}, ${intersection.y.toFixed(4)}, ${intersection.z.toFixed(4)})`);
                    console.log(`  上次位置: (${this.dragCurrentPosition.x.toFixed(4)}, ${this.dragCurrentPosition.y.toFixed(4)}, ${this.dragCurrentPosition.z.toFixed(4)})`);
                    console.log(`  鼠标移动: (${deltaPos.x.toFixed(4)}, ${deltaPos.y.toFixed(4)}, ${deltaPos.z.toFixed(4)})`);
                    console.log(`  箭头方向: (${worldAxisDir.x.toFixed(4)}, ${worldAxisDir.y.toFixed(4)}, ${worldAxisDir.z.toFixed(4)})`);
                    console.log(`  投影长度: ${projectionLength.toFixed(4)}`);
                    console.log(`  计算位移: (${projectedDelta.x.toFixed(4)}, ${projectedDelta.y.toFixed(4)}, ${projectedDelta.z.toFixed(4)})`);
                    this._debugLogged = true;
                    this._lastDebugTime = Date.now();
                }
                
                // 更新拖动当前位置
                this.dragCurrentPosition = intersection.clone();
                
                // 使用雅可比伪逆更新关节角度
                this.updateJointsFromEndEffectorDelta(projectedDelta);
            }
        } else if (this.arrowType === 'rotation') {
            // 旋转箭头拖动
            const intersection = new THREE.Vector3();
            const hasIntersection = this.raycaster.ray.intersectPlane(this.dragPlane, intersection);
            
            if (hasIntersection) {
                // 计算旋转角度
                const axisDir = this.getAxisDirection(this.arrowAxis);
                const worldAxisDir = axisDir.clone().applyQuaternion(this.endEffectorGizmo.quaternion);
                
                // 计算从末端执行器到交点的向量
                const toIntersection = intersection.sub(this.endEffector.position).normalize();
                const toPrevious = this.dragCurrentPosition.sub(this.endEffector.position).normalize();
                
                // 计算角度变化（使用叉积和点积）
                const cross = new THREE.Vector3().crossVectors(toPrevious, toIntersection);
                const dot = toPrevious.dot(toIntersection);
                const angle = Math.atan2(cross.dot(worldAxisDir), dot);
                
                // 将角度转换为旋转四元数
                const rotationQuat = new THREE.Quaternion().setFromAxisAngle(worldAxisDir, angle);
                
                // 计算旋转后的末端位置变化
                const currentPos = this.endEffector.position.clone();
                const rotatedPos = currentPos.clone().applyQuaternion(rotationQuat);
                const deltaPos = rotatedPos.sub(currentPos);
                
                this.dragCurrentPosition = intersection.clone();
                
                // 使用雅可比伪逆更新关节角度
                this.updateJointsFromEndEffectorDelta(deltaPos);
            }
        }
    }
    
    /**
     * 处理拖动末端执行器的鼠标释放事件
     */
    handleDragEndEffectorMouseUp(e) {
        if (!this.dragEndEffectorMode || e.button !== 0) return;
        
        // 取消高亮
        if (this.selectedArrow) {
            this.highlightArrow(this.selectedArrow, false);
        }
        
        this.isDraggingEndEffector = false;
        this.selectedArrow = null;
        this.arrowType = null;
        this.arrowAxis = null;
        this.selectedArrowDirection = null;  // 清除方向信息
        this.worldAxisDirection = null;  // 清除锁定的方向
        this.lockedPlaneNormal = null;  // 清除锁定的平面法向量
        this.dragPlane = null;
        this._debugLogged = false;  // 重置调试标志
    }
    
    /**
     * 更新鼠标位置（用于 Raycaster）
     */
    updateMousePosition(e) {
        const rect = this.canvas.getBoundingClientRect();
        this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
        this.raycaster.setFromCamera(this.mouse, this.camera);
    }
    
    /**
     * 沿指定轴方向移动双臂绝对位姿（示教器模式，使用零空间保持相对位姿不变）
     * @param {string} axis - 轴名称：'x', 'y', 或 'z'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    moveDualArmAbsolutePoseAlongAxis(axis, direction) {
        if (!this.dualArmMode) {
            console.warn('双臂绝对位姿控制仅支持双臂模式');
            return;
        }
        
        if (!window.dualArmSystem) {
            console.warn('双臂系统未初始化');
            return;
        }
        
        if (!window.jointAngles1 || !window.jointAngles2 || 
            window.jointAngles1.length === 0 || window.jointAngles2.length === 0) {
            console.warn('关节角度未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            // 计算移动步长（单位：米）
            const stepSize = 0.01; // 1cm per step
            
            // 合并关节角度：[robot1_joints..., robot2_joints...]
            const allJointAngles = [...window.jointAngles1, ...window.jointAngles2];
            
            // 获取当前绝对位姿
            const dqAbsoluteCurrentArray = window.dualArmSystem.getAbsolutePose(allJointAngles);
            if (!dqAbsoluteCurrentArray || dqAbsoluteCurrentArray.length !== 8) {
                console.error('无法获取当前绝对位姿');
                return;
            }
            
            const dqAbsoluteCurrent = window.DQModule.DQWrapper.createFromArray(dqAbsoluteCurrentArray);
            
            // 从双四元数中提取旋转四元数（用于确定绝对位姿坐标系）
            const rotationArray = window.DQModule.DQWrapper.getRotation(dqAbsoluteCurrent);
            const qw = rotationArray[0] || 1;
            const qx = rotationArray[1] || 0;
            const qy = rotationArray[2] || 0;
            const qz = rotationArray[3] || 0;
            
            // 创建 THREE.js 四元数
            const absolutePoseQuaternion = new THREE.Quaternion(qx, qy, qz, qw);
            
            // 局部轴方向（在绝对位姿坐标系中）
            const localAxisDir = new THREE.Vector3();
            if (axis === 'x') localAxisDir.set(1, 0, 0);
            else if (axis === 'y') localAxisDir.set(0, 1, 0);
            else if (axis === 'z') localAxisDir.set(0, 0, 1);
            else {
                console.error('无效的轴名称:', axis);
                return;
            }
            
            // 转换到世界坐标系
            const worldAxisDir = localAxisDir.clone().applyQuaternion(absolutePoseQuaternion);
            
            // 根据方向确定移动方向
            const directionSign = direction === 'positive' ? 1 : -1;
            const delta = worldAxisDir.multiplyScalar(stepSize * directionSign);
            
            // 使用零空间方法更新关节角度（保持相对位姿不变）
            this.updateDualArmJointsFromAbsolutePoseDelta(delta, allJointAngles);
        } catch (error) {
            console.error('移动双臂绝对位姿失败:', error);
        }
    }
    
    /**
     * 绕指定轴旋转双臂绝对位姿（示教器模式，使用零空间保持相对位姿不变）
     * @param {string} axis - 旋转轴名称：'rx', 'ry', 或 'rz'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    rotateDualArmAbsolutePoseAroundAxis(axis, direction) {
        if (!this.dualArmMode) {
            console.warn('双臂绝对位姿控制仅支持双臂模式');
            return;
        }
        
        if (!window.dualArmSystem) {
            console.warn('双臂系统未初始化');
            return;
        }
        
        if (!window.jointAngles1 || !window.jointAngles2 || 
            window.jointAngles1.length === 0 || window.jointAngles2.length === 0) {
            console.warn('关节角度未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            // 旋转角度（弧度）
            const angle = Math.PI / 128;
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            
            // 合并关节角度
            const allJointAngles = [...window.jointAngles1, ...window.jointAngles2];
            
            // 获取当前绝对位姿
            const dqAbsoluteCurrentArray = window.dualArmSystem.getAbsolutePose(allJointAngles);
            const dqAbsoluteCurrent = window.DQModule.DQWrapper.createFromArray(dqAbsoluteCurrentArray);
            
            // 创建旋转双四元数
            let rotationDQ;
            if (axis === 'rx') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 1, 0, 0);
            } else if (axis === 'ry') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 1, 0);
            } else if (axis === 'rz') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 0, 1);
            } else {
                console.error('无效的旋转轴名称:', axis);
                return;
            }
            
            // 计算目标绝对位姿：dq_target = rotation * dq_current
            const dqAbsoluteTarget = window.DQModule.DQWrapper.multiply(dqAbsoluteCurrent, rotationDQ);
            const dqAbsoluteTargetArray = window.DQModule.DQWrapper.toArray(dqAbsoluteTarget);
            
            // 使用零空间方法更新关节角度（保持相对位姿不变）
            this.updateDualArmJointsFromAbsolutePoseTarget(dqAbsoluteTargetArray, allJointAngles);
        } catch (error) {
            console.error('旋转双臂绝对位姿失败:', error);
        }
    }
    
    /**
     * 使用零空间方法从绝对位姿增量更新双臂关节角度（保持相对位姿不变）
     * 参考 MATLAB cdts_bucket_kuka.m：
     * - 使用完整雅可比矩阵：[J_abs; J_rel]
     * - 误差向量：[err_abs; 0]（相对位姿误差为0）
     * - 更新：q = q + pinv(J) * damping * err
     * 
     * @param {THREE.Vector3} delta - 绝对位姿的位移（世界坐标系，单位：米）
     * @param {number[]} currentJointAngles - 当前关节角度 [robot1_joints..., robot2_joints...]
     */
    updateDualArmJointsFromAbsolutePoseDelta(delta, currentJointAngles) {
        if (!window.dualArmSystem || !currentJointAngles || currentJointAngles.length === 0) return;
        
        try {
            // 获取当前绝对位姿和相对位姿
            const dqAbsoluteCurrentArray = window.dualArmSystem.getAbsolutePose(currentJointAngles);
            const dqRelativeCurrentArray = window.dualArmSystem.getRelativePose(currentJointAngles);
            
            // 创建位移的平移双四元数
            const dqDelta = window.DQModule.DQWrapper.translation(delta.x, delta.y, delta.z);
            const dqAbsoluteCurrent = window.DQModule.DQWrapper.createFromArray(dqAbsoluteCurrentArray);
            
            // 计算目标绝对位姿
            const dqAbsoluteTarget = window.DQModule.DQWrapper.multiply(dqDelta, dqAbsoluteCurrent);
            const dqAbsoluteTargetArray = window.DQModule.DQWrapper.toArray(dqAbsoluteTarget);
            
            // 使用迭代求解
            this.updateDualArmJointsFromAbsolutePoseTarget(dqAbsoluteTargetArray, currentJointAngles, dqRelativeCurrentArray);
        } catch (error) {
            console.error('更新双臂关节角度失败:', error);
        }
    }
    
    /**
     * 使用零空间方法从绝对位姿目标更新双臂关节角度（保持相对位姿不变）
     * 
     * @param {number[]} dqAbsoluteTargetArray - 目标绝对位姿（8元素双四元数）
     * @param {number[]} currentJointAngles - 当前关节角度
     * @param {number[]} dqRelativeCurrentArray - 当前相对位姿（可选，如果不提供则从当前关节角度计算）
     */
    updateDualArmJointsFromAbsolutePoseTarget(dqAbsoluteTargetArray, currentJointAngles, dqRelativeCurrentArray = null) {
        if (!window.dualArmSystem || !currentJointAngles || currentJointAngles.length === 0) return;
        
        try {
            // 获取当前相对位姿（如果未提供）
            if (!dqRelativeCurrentArray) {
                dqRelativeCurrentArray = window.dualArmSystem.getRelativePose(currentJointAngles);
            }
            
            // 迭代求解参数
            const epsilon = 0.0000001;
            const maxIterations = 50;
            const damping = 0.5;
            
            let q = [...currentJointAngles];
            let errNorm = epsilon + 1;
            let iteration = 0;
            
            while (errNorm > epsilon && iteration < maxIterations) {
                // 获取当前绝对位姿和相对位姿
                const dqAbsoluteCurrentIterArray = window.dualArmSystem.getAbsolutePose(q);
                const dqRelativeCurrentIterArray = window.dualArmSystem.getRelativePose(q);
                
                // 计算误差向量
                // 绝对位姿误差（8元素）
                const errAbsolute = [];
                for (let i = 0; i < 8; i++) {
                    errAbsolute.push(dqAbsoluteTargetArray[i] - dqAbsoluteCurrentIterArray[i]);
                }
                
                // 相对位姿误差（保持为0，即保持相对位姿不变）
                const errRelative = [];
                for (let i = 0; i < 8; i++) {
                    errRelative.push(dqRelativeCurrentArray[i] - dqRelativeCurrentIterArray[i]);
                }
                
                // 组合误差向量：[err_abs; err_rel]（16元素）
                const err = [...errAbsolute, ...errRelative];
                
                // 计算误差范数
                errNorm = Math.sqrt(err.reduce((sum, e) => sum + e * e, 0));
                
                if (errNorm <= epsilon) {
                    break;
                }
                
                // 获取雅可比矩阵
                // 绝对位姿雅可比（8xN）
                const jacobianAbsolute = window.dualArmSystem.getAbsolutePoseJacobian(q);
                // 相对位姿雅可比（8xN）
                const jacobianRelative = window.dualArmSystem.getRelativePoseJacobian(q);
                
                // 组合雅可比矩阵：[J_abs; J_rel]（16xN）
                const jacobian = [];
                for (let i = 0; i < jacobianAbsolute.length; i++) {
                    jacobian.push([...jacobianAbsolute[i]]);
                }
                for (let i = 0; i < jacobianRelative.length; i++) {
                    jacobian.push([...jacobianRelative[i]]);
                }
                
                // 计算雅可比伪逆
                const jacobianPseudoInv = this.pseudoInverse(jacobian);
                
                // 计算关节角度增量：dq = pinv(J) * damping * err
                const deltaJoints = [];
                for (let i = 0; i < jacobianPseudoInv.length; i++) {
                    let sum = 0;
                    for (let j = 0; j < err.length; j++) {
                        sum += jacobianPseudoInv[i][j] * err[j] * damping;
                    }
                    deltaJoints.push(sum);
                }
                
                // 更新关节角度
                for (let i = 0; i < q.length && i < deltaJoints.length; i++) {
                    q[i] += deltaJoints[i];
                }
                
                iteration++;
            }
            
            // 分离关节角度
            const robot1JointCount = window.jointAngles1.length;
            const robot2JointCount = window.jointAngles2.length;
            
            // 限制关节角度
            if (window.customRobotConfig) {
                const jointNames1 = window.customRobotConfig?.robot1?.joint_chain?.joints || [];
                const jointNames2 = window.customRobotConfig?.robot2?.joint_chain?.joints || [];
                
                const q1 = q.slice(0, robot1JointCount);
                const q2 = q.slice(robot1JointCount, robot1JointCount + robot2JointCount);
                
                if (window.clampJointAngles) {
                    const clampedQ1 = window.clampJointAngles(q1, jointNames1, this.robot1URDF, window.customRobotConfig?.robot1);
                    const clampedQ2 = window.clampJointAngles(q2, jointNames2, this.robot2URDF, window.customRobotConfig?.robot2);
                    q = [...clampedQ1, ...clampedQ2];
                }
            }
            
            // 更新全局关节角度
            const q1 = q.slice(0, robot1JointCount);
            const q2 = q.slice(robot1JointCount, robot1JointCount + robot2JointCount);
            
            for (let i = 0; i < window.jointAngles1.length && i < q1.length; i++) {
                window.jointAngles1[i] = q1[i];
            }
            for (let i = 0; i < window.jointAngles2.length && i < q2.length; i++) {
                window.jointAngles2[i] = q2[i];
            }
            
            // 同步到 main.js
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            
            // 更新机器人位姿
            if (window.updateRobotPose) {
                window.updateRobotPose();
            }
        } catch (error) {
            console.error('更新双臂关节角度失败:', error);
        }
    }
    
    /**
     * 沿指定轴方向移动双臂相对位姿（示教器模式，使用零空间保持绝对位姿不变）
     * @param {string} axis - 轴名称：'x', 'y', 或 'z'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    moveDualArmRelativePoseAlongAxis(axis, direction) {
        if (!this.dualArmMode) {
            console.warn('双臂相对位姿控制仅支持双臂模式');
            return;
        }
        
        if (!window.dualArmSystem) {
            console.warn('双臂系统未初始化');
            return;
        }
        
        if (!window.jointAngles1 || !window.jointAngles2 || 
            window.jointAngles1.length === 0 || window.jointAngles2.length === 0) {
            console.warn('关节角度未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            // 计算移动步长（单位：米）
            const stepSize = 0.01; // 1cm per step
            
            // 合并关节角度：[robot1_joints..., robot2_joints...]
            const allJointAngles = [...window.jointAngles1, ...window.jointAngles2];
            
            // 获取当前相对位姿
            const dqRelativeCurrentArray = window.dualArmSystem.getRelativePose(allJointAngles);
            if (!dqRelativeCurrentArray || dqRelativeCurrentArray.length !== 8) {
                console.error('无法获取当前相对位姿');
                return;
            }
            
            const dqRelativeCurrent = window.DQModule.DQWrapper.createFromArray(dqRelativeCurrentArray);
            
            // 从双四元数中提取旋转四元数（用于确定相对位姿坐标系）
            const rotationArray = window.DQModule.DQWrapper.getRotation(dqRelativeCurrent);
            const qw = rotationArray[0] || 1;
            const qx = rotationArray[1] || 0;
            const qy = rotationArray[2] || 0;
            const qz = rotationArray[3] || 0;
            
            // 创建 THREE.js 四元数
            const relativePoseQuaternion = new THREE.Quaternion(qx, qy, qz, qw);
            
            // 局部轴方向（在相对位姿坐标系中）
            const localAxisDir = new THREE.Vector3();
            if (axis === 'x') localAxisDir.set(1, 0, 0);
            else if (axis === 'y') localAxisDir.set(0, 1, 0);
            else if (axis === 'z') localAxisDir.set(0, 0, 1);
            else {
                console.error('无效的轴名称:', axis);
                return;
            }
            
            // 转换到世界坐标系
            const worldAxisDir = localAxisDir.clone().applyQuaternion(relativePoseQuaternion);
            
            // 根据方向确定移动方向
            const directionSign = direction === 'positive' ? 1 : -1;
            const delta = worldAxisDir.multiplyScalar(stepSize * directionSign);
            
            // 使用零空间方法更新关节角度（保持绝对位姿不变）
            this.updateDualArmJointsFromRelativePoseDelta(delta, allJointAngles);
        } catch (error) {
            console.error('移动双臂相对位姿失败:', error);
        }
    }
    
    /**
     * 绕指定轴旋转双臂相对位姿（示教器模式，使用零空间保持绝对位姿不变）
     * @param {string} axis - 旋转轴名称：'rx', 'ry', 或 'rz'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    rotateDualArmRelativePoseAroundAxis(axis, direction) {
        if (!this.dualArmMode) {
            console.warn('双臂相对位姿控制仅支持双臂模式');
            return;
        }
        
        if (!window.dualArmSystem) {
            console.warn('双臂系统未初始化');
            return;
        }
        
        if (!window.jointAngles1 || !window.jointAngles2 || 
            window.jointAngles1.length === 0 || window.jointAngles2.length === 0) {
            console.warn('关节角度未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            // 旋转角度（弧度）
            const angle = Math.PI / 128;
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            
            // 合并关节角度
            const allJointAngles = [...window.jointAngles1, ...window.jointAngles2];
            
            // 获取当前相对位姿
            const dqRelativeCurrentArray = window.dualArmSystem.getRelativePose(allJointAngles);
            const dqRelativeCurrent = window.DQModule.DQWrapper.createFromArray(dqRelativeCurrentArray);
            
            // 创建旋转双四元数
            let rotationDQ;
            if (axis === 'rx') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 1, 0, 0);
            } else if (axis === 'ry') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 1, 0);
            } else if (axis === 'rz') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 0, 1);
            } else {
                console.error('无效的旋转轴名称:', axis);
                return;
            }
            
            // 计算目标相对位姿：dq_target = rotation * dq_current
            const dqRelativeTarget = window.DQModule.DQWrapper.multiply(rotationDQ, dqRelativeCurrent);
            const dqRelativeTargetArray = window.DQModule.DQWrapper.toArray(dqRelativeTarget);
            
            // 使用零空间方法更新关节角度（保持绝对位姿不变）
            this.updateDualArmJointsFromRelativePoseTarget(dqRelativeTargetArray, allJointAngles);
        } catch (error) {
            console.error('旋转双臂相对位姿失败:', error);
        }
    }
    
    /**
     * 使用零空间方法从相对位姿增量更新双臂关节角度（保持绝对位姿不变）
     * 
     * @param {THREE.Vector3} delta - 相对位姿的位移（世界坐标系，单位：米）
     * @param {number[]} currentJointAngles - 当前关节角度 [robot1_joints..., robot2_joints...]
     */
    updateDualArmJointsFromRelativePoseDelta(delta, currentJointAngles) {
        if (!window.dualArmSystem || !currentJointAngles || currentJointAngles.length === 0) return;
        
        try {
            // 获取当前绝对位姿和相对位姿
            const dqAbsoluteCurrentArray = window.dualArmSystem.getAbsolutePose(currentJointAngles);
            const dqRelativeCurrentArray = window.dualArmSystem.getRelativePose(currentJointAngles);
            
            // 创建位移的平移双四元数
            const dqDelta = window.DQModule.DQWrapper.translation(delta.x, delta.y, delta.z);
            const dqRelativeCurrent = window.DQModule.DQWrapper.createFromArray(dqRelativeCurrentArray);
            
            // 计算目标相对位姿
            const dqRelativeTarget = window.DQModule.DQWrapper.multiply(dqDelta, dqRelativeCurrent);
            const dqRelativeTargetArray = window.DQModule.DQWrapper.toArray(dqRelativeTarget);
            
            // 使用迭代求解
            this.updateDualArmJointsFromRelativePoseTarget(dqRelativeTargetArray, currentJointAngles, dqAbsoluteCurrentArray);
        } catch (error) {
            console.error('更新双臂关节角度失败:', error);
        }
    }
    
    /**
     * 使用零空间方法从相对位姿目标更新双臂关节角度（保持绝对位姿不变）
     * 
     * @param {number[]} dqRelativeTargetArray - 目标相对位姿（8元素双四元数）
     * @param {number[]} currentJointAngles - 当前关节角度
     * @param {number[]} dqAbsoluteCurrentArray - 当前绝对位姿（可选，如果不提供则从当前关节角度计算）
     */
    updateDualArmJointsFromRelativePoseTarget(dqRelativeTargetArray, currentJointAngles, dqAbsoluteCurrentArray = null) {
        if (!window.dualArmSystem || !currentJointAngles || currentJointAngles.length === 0) return;
        
        try {
            // 获取当前绝对位姿（如果未提供）
            if (!dqAbsoluteCurrentArray) {
                dqAbsoluteCurrentArray = window.dualArmSystem.getAbsolutePose(currentJointAngles);
            }
            
            // 迭代求解参数
            const epsilon = 0.001;
            const maxIterations = 50;
            const damping = 0.5;
            
            let q = [...currentJointAngles];
            let errNorm = epsilon + 1;
            let iteration = 0;
            
            while (errNorm > epsilon && iteration < maxIterations) {
                // 获取当前绝对位姿和相对位姿
                const dqAbsoluteCurrentIterArray = window.dualArmSystem.getAbsolutePose(q);
                const dqRelativeCurrentIterArray = window.dualArmSystem.getRelativePose(q);
                
                // 计算误差向量
                // 绝对位姿误差（保持为0，即保持绝对位姿不变）
                const errAbsolute = [];
                for (let i = 0; i < 8; i++) {
                    errAbsolute.push(dqAbsoluteCurrentArray[i] - dqAbsoluteCurrentIterArray[i]);
                }
                
                // 相对位姿误差（8元素）
                const errRelative = [];
                for (let i = 0; i < 8; i++) {
                    errRelative.push(dqRelativeTargetArray[i] - dqRelativeCurrentIterArray[i]);
                }
                
                // 组合误差向量：[err_abs; err_rel]（16元素）
                const err = [...errAbsolute, ...errRelative];
                
                // 计算误差范数
                errNorm = Math.sqrt(err.reduce((sum, e) => sum + e * e, 0));
                
                if (errNorm <= epsilon) {
                    break;
                }
                
                // 获取雅可比矩阵
                // 绝对位姿雅可比（8xN）
                const jacobianAbsolute = window.dualArmSystem.getAbsolutePoseJacobian(q);
                // 相对位姿雅可比（8xN）
                const jacobianRelative = window.dualArmSystem.getRelativePoseJacobian(q);
                
                // 组合雅可比矩阵：[J_abs; J_rel]（16xN）
                const jacobian = [];
                for (let i = 0; i < jacobianAbsolute.length; i++) {
                    jacobian.push([...jacobianAbsolute[i]]);
                }
                for (let i = 0; i < jacobianRelative.length; i++) {
                    jacobian.push([...jacobianRelative[i]]);
                }
                
                // 计算雅可比伪逆
                const jacobianPseudoInv = this.pseudoInverse(jacobian);
                
                // 计算关节角度增量：dq = pinv(J) * damping * err
                const deltaJoints = [];
                for (let i = 0; i < jacobianPseudoInv.length; i++) {
                    let sum = 0;
                    for (let j = 0; j < err.length; j++) {
                        sum += jacobianPseudoInv[i][j] * err[j] * damping;
                    }
                    deltaJoints.push(sum);
                }
                
                // 更新关节角度
                for (let i = 0; i < q.length && i < deltaJoints.length; i++) {
                    q[i] += deltaJoints[i];
                }
                
                iteration++;
            }
            
            // 分离关节角度
            const robot1JointCount = window.jointAngles1.length;
            const robot2JointCount = window.jointAngles2.length;
            
            // 限制关节角度
            if (window.customRobotConfig) {
                const jointNames1 = window.customRobotConfig?.robot1?.joint_chain?.joints || [];
                const jointNames2 = window.customRobotConfig?.robot2?.joint_chain?.joints || [];
                
                const q1 = q.slice(0, robot1JointCount);
                const q2 = q.slice(robot1JointCount, robot1JointCount + robot2JointCount);
                
                if (window.clampJointAngles) {
                    const clampedQ1 = window.clampJointAngles(q1, jointNames1, this.robot1URDF, window.customRobotConfig?.robot1);
                    const clampedQ2 = window.clampJointAngles(q2, jointNames2, this.robot2URDF, window.customRobotConfig?.robot2);
                    q = [...clampedQ1, ...clampedQ2];
                }
            }
            
            // 更新全局关节角度
            const q1 = q.slice(0, robot1JointCount);
            const q2 = q.slice(robot1JointCount, robot1JointCount + robot2JointCount);
            
            for (let i = 0; i < window.jointAngles1.length && i < q1.length; i++) {
                window.jointAngles1[i] = q1[i];
            }
            for (let i = 0; i < window.jointAngles2.length && i < q2.length; i++) {
                window.jointAngles2[i] = q2[i];
            }
            
            // 同步到 main.js
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            
            // 更新机器人位姿
            if (window.updateRobotPose) {
                window.updateRobotPose();
            }
        } catch (error) {
            console.error('更新双臂关节角度失败:', error);
        }
    }
    
    /**
     * 沿指定轴方向移动末端执行器（示教器模式）
     * @param {string} axis - 轴名称：'x', 'y', 或 'z'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    moveEndEffectorAlongAxis(axis, direction) {
        if (this.dualArmMode || !this.endEffector) {
            console.warn('示教器控制仅支持单臂模式');
            return;
        }
        
        if (!window.currentRobot || !window.jointAngles || window.jointAngles.length === 0) {
            console.warn('机器人未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            // 计算移动步长（单位：米）
            const stepSize = 0.01; // 1cm per step
            
            // 直接从 DQ Robotics 获取当前末端执行器位姿（而不是依赖视觉化的endEffector）
            const dqCurrentArray = window.currentRobot.fkm(window.jointAngles);
            if (!dqCurrentArray || dqCurrentArray.length !== 8) {
                console.error('无法获取当前位姿');
                return;
            }
            
            const dqCurrent = window.DQModule.DQWrapper.createFromArray(dqCurrentArray);
            
            // 从双四元数中提取旋转四元数（用于确定末端执行器坐标系）
            const rotationArray = window.DQModule.DQWrapper.getRotation(dqCurrent);
            // rotationArray格式：[w, x, y, z]
            const qw = rotationArray[0] || 1;
            const qx = rotationArray[1] || 0;
            const qy = rotationArray[2] || 0;
            const qz = rotationArray[3] || 0;
            
            // 创建 THREE.js 四元数（注意：THREE.js 四元数格式是 [x, y, z, w]）
            const endEffectorQuaternion = new THREE.Quaternion(qx, qy, qz, qw);
            
            // 局部轴方向（在末端执行器坐标系中）
            const localAxisDir = new THREE.Vector3();
            if (axis === 'x') localAxisDir.set(1, 0, 0);
            else if (axis === 'y') localAxisDir.set(0, 1, 0);
            else if (axis === 'z') localAxisDir.set(0, 0, 1);
            else {
                console.error('无效的轴名称:', axis);
                return;
            }
            
            // 转换到世界坐标系
            const worldAxisDir = localAxisDir.clone().applyQuaternion(endEffectorQuaternion);
            
            // 根据方向确定移动方向
            const directionSign = direction === 'positive' ? 1 : -1;
            const delta = worldAxisDir.multiplyScalar(stepSize * directionSign);
            
            // 使用现有的逆运动学方法更新关节角度
            this.updateJointsFromEndEffectorDelta(delta);
        } catch (error) {
            console.error('移动末端执行器失败:', error);
        }
    }
    
    /**
     * 绕指定轴旋转末端执行器（示教器模式）
     * 参考 MATLAB: dqad = DQ([cos(pi/16);0;sin(pi/16);0]) .* dqad_ant;
     * 
     * @param {string} axis - 旋转轴名称：'rx', 'ry', 或 'rz'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    rotateEndEffectorAroundAxis(axis, direction) {
        if (this.dualArmMode || !this.endEffector) {
            console.warn('示教器控制仅支持单臂模式');
            return;
        }
        
        if (!window.currentRobot || !window.jointAngles || window.jointAngles.length === 0) {
            console.warn('机器人未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            // 旋转角度（弧度），pi/16 ≈ 11.25度
            const angle = Math.PI / 128;
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            
            // 获取当前末端执行器位姿
            const dqCurrentArray = window.currentRobot.fkm(window.jointAngles);
            const dqCurrent = window.DQModule.DQWrapper.createFromArray(dqCurrentArray);
            
            // 创建旋转双四元数（在末端执行器局部坐标系中）
            // 根据用户提供的MATLAB代码：DQ([cos(pi/16);0;sin(pi/16);0])
            // 这是绕Y轴旋转，格式为 [w, x, y, z]
            let rotationDQ;
            if (axis === 'rx') {
                // 绕X轴旋转：[cos(θ), sin(θ), 0, 0]
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 1, 0, 0);
            } else if (axis === 'ry') {
                // 绕Y轴旋转：[cos(θ), 0, sin(θ), 0]
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 1, 0);
            } else if (axis === 'rz') {
                // 绕Z轴旋转：[cos(θ), 0, 0, sin(θ)]
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 0, 1);
            } else {
                console.error('无效的旋转轴名称:', axis);
                return;
            }
            
            // 计算目标位姿：dq_target = dq_rotation * dq_current
            // 参考 MATLAB: dqad = DQ([cos(pi/16);0;sin(pi/16);0]) .* dqad_ant;
            // 注意：在DQ中，旋转应该在左侧：rotation * current（先旋转再应用当前位姿）
            const dqTarget = window.DQModule.DQWrapper.multiply(dqCurrent,rotationDQ);
            const dqTargetArray = window.DQModule.DQWrapper.toArray(dqTarget);
            
            // 迭代求解逆运动学（参考 MATLAB 的 while 循环）
            const epsilon = 0.00001;  // 停止条件：误差范数小于此值
            const maxIterations = 50;  // 最大迭代次数
            const damping = 0.5;  // 阻尼系数（与MATLAB一致）
            
            // 使用局部变量进行迭代
            let q = [...window.jointAngles];
            let errNorm = epsilon + 1;
            let iteration = 0;
            
            while (errNorm > epsilon && iteration < maxIterations) {
                // 获取当前位姿
                const dqCurrentIterArray = window.currentRobot.fkm(q);
                const dqCurrentIter = window.DQModule.DQWrapper.createFromArray(dqCurrentIterArray);
                
                // 计算误差向量
                const err = [];
                for (let i = 0; i < 8; i++) {
                    err.push(dqTargetArray[i] - dqCurrentIterArray[i]);
                }
                
                // 计算误差范数
                errNorm = Math.sqrt(err.reduce((sum, e) => sum + e * e, 0));
                
                if (errNorm <= epsilon) {
                    break;  // 误差足够小，退出循环
                }
                
                // 获取雅可比矩阵
                const jacobian = window.currentRobot.poseJacobian(q);
                if (!jacobian || jacobian.length !== 8) {
                    console.error('雅可比矩阵维度错误');
                    return;
                }
                
                // 计算雅可比伪逆
                const jacobianPseudoInv = this.pseudoInverse(jacobian);
                
                // 计算关节角度增量
                const deltaJoints = [];
                for (let i = 0; i < jacobianPseudoInv.length; i++) {
                    let sum = 0;
                    for (let j = 0; j < err.length; j++) {
                        sum += jacobianPseudoInv[i][j] * err[j] * damping;
                    }
                    deltaJoints.push(sum);
                }
                
                // 更新关节角度
                for (let i = 0; i < q.length && i < deltaJoints.length; i++) {
                    q[i] += deltaJoints[i];
                }
                
                iteration++;
            }
            
            // 限制关节角度在限位范围内
            if (window.customRobotConfig && window.customRobotConfig.joint_chain) {
                const jointNames = window.customRobotConfig.joint_chain.joints || [];
                q = window.clampJointAngles(q, jointNames, this.urdfRobot, window.customRobotConfig);
            }
            
            // 更新全局关节角度
            if (!window.jointAngles || window.jointAngles.length !== q.length) {
                if (window.updateGlobalVariables) {
                    window.updateGlobalVariables();
                }
            }
            
            // 更新关节角度
            for (let i = 0; i < window.jointAngles.length && i < q.length; i++) {
                window.jointAngles[i] = q[i];
            }
            
            // 同步到 main.js
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            
            // 更新机器人位姿
            if (window.updateRobotPose) {
                window.updateRobotPose();
            }
            
            // 确保拖动控制跟随末端执行器
            this.updateGizmoPosition();
            
        } catch (error) {
            console.error('旋转末端执行器失败:', error);
        }
    }
    
    /**
     * 使用双四元数方式和雅可比伪逆从末端执行器位移更新关节角度
     * 
     * 参考 MATLAB 代码 cdts_bucket_kuka.m 的实现方式：
     * 1. 获取当前末端执行器位姿（双四元数）：dq_current = robot.fkm(jointAngles)
     * 2. 计算位移的平移双四元数：dq_delta = 1 + DQ.E*0.5*DQ([0, dx, dy, dz])
     *    使用 DQ.translation(dx, dy, dz) 创建：1 + 0.5*E*DQ(0, dx, dy, dz)
     * 3. 计算目标位姿：dq_target = dq_delta * dq_current（在世界坐标系中）
     * 4. 计算8元素误差向量：err = vec8(dq_target) - vec8(dq_current)
     * 5. 获取完整的8xN位姿雅可比矩阵（8行对应双四元数的8个元素，N列对应关节数）
     * 6. 使用伪逆更新关节角度：q = q + pinv(J) * 0.5 * err
     * 
     * 注意：poseJacobian 返回的是 8xN 矩阵，前4行是旋转四元数的雅可比，后4行是平移的雅可比（与旋转耦合）
     * 
     * @param {THREE.Vector3} delta - 末端执行器的位移（世界坐标系，单位：米）
     */
    updateJointsFromEndEffectorDelta(delta) {
        if (!window.currentRobot || !window.jointAngles || window.jointAngles.length === 0) return;
        
        try {
            // 检查 DQModule 是否可用
            if (!window.DQModule || !window.DQModule.DQWrapper) {
                console.error('DQ Robotics 模块未加载');
                return;
            }
            
            // 1. 获取当前末端执行器位姿（双四元数，8元素数组）
            // 参考 MATLAB: dqad_ant = two_arms.absolute_pose(q);
            const dqCurrentArray = window.currentRobot.fkm(window.jointAngles);
            if (!dqCurrentArray || dqCurrentArray.length !== 8) {
                console.error('无法获取当前位姿');
                return;
            }
            
            // 2. 创建位移的平移双四元数：dq_delta = 1 + DQ.E*0.5*DQ([0, dx, dy, dz])
            // 参考 MATLAB: dqad = (1+DQ.E*0.5*(-0.1*DQ.i-0.1*DQ.j-0.1*DQ.k)) * dqad_ant
            // 在 JavaScript 中，使用 DQWrapper.translation(dx, dy, dz) 创建
            const dqDelta = window.DQModule.DQWrapper.translation(delta.x, delta.y, delta.z);
            const dqCurrent = window.DQModule.DQWrapper.createFromArray(dqCurrentArray);
            
            // 3. 计算目标位姿：dq_target = dq_delta * dq_current
            // 注意：在世界坐标系中，先应用平移再应用当前位姿
            // 由于 dq_delta 是纯平移（旋转部分为单位四元数），dq_target 的旋转部分与 dq_current 相同
            const dqTarget = window.DQModule.DQWrapper.multiply(dqDelta, dqCurrent);
            
            // 4. 保存目标位姿数组
            const dqTargetArray = window.DQModule.DQWrapper.toArray(dqTarget);
            
            // 5. 迭代求解逆运动学（参考 MATLAB 的 while 循环）
            const epsilon = 0.001;  // 停止条件：误差范数小于此值
            const maxIterations = 50;  // 最大迭代次数，防止无限循环
            const damping = 0.5;  // 阻尼系数（与MATLAB一致）
            
            // 使用局部变量进行迭代，避免频繁修改全局 jointAngles
            let q = [...window.jointAngles];
            let errNorm = epsilon + 1;
            let iteration = 0;
            
            while (errNorm > epsilon && iteration < maxIterations) {
                // 获取当前位姿
                const dqCurrentIterArray = window.currentRobot.fkm(q);
                
                // 计算误差向量
                const err = [];
                for (let i = 0; i < 8; i++) {
                    err.push(dqTargetArray[i] - dqCurrentIterArray[i]);
                }
                
                // 计算误差范数
                errNorm = Math.sqrt(err.reduce((sum, e) => sum + e * e, 0));
                
                if (errNorm <= epsilon) {
                    break;  // 误差足够小，退出循环
                }
                
                // 获取雅可比矩阵
                const jacobian = window.currentRobot.poseJacobian(q);
                if (!jacobian || jacobian.length !== 8) {
                    console.error('雅可比矩阵维度错误');
                    return;
                }
                
                // 计算雅可比伪逆
                const jacobianPseudoInv = this.pseudoInverse(jacobian);
                
                // 计算关节角度增量
                const deltaJoints = [];
                for (let i = 0; i < jacobianPseudoInv.length; i++) {
                    let sum = 0;
                    for (let j = 0; j < err.length; j++) {
                        sum += jacobianPseudoInv[i][j] * err[j] * damping;
                    }
                    deltaJoints.push(sum);
                }
                
                // 更新关节角度
                for (let i = 0; i < q.length && i < deltaJoints.length; i++) {
                    q[i] += deltaJoints[i];
                }
                
                iteration++;
            }
            
            // 6. 限制关节角度在限位范围内
            if (window.customRobotConfig && window.customRobotConfig.joint_chain) {
                const jointNames = window.customRobotConfig.joint_chain.joints || [];
                q = window.clampJointAngles(q, jointNames, this.urdfRobot, window.customRobotConfig);
            }
            
            // 7. 更新全局关节角度
            if (!window.jointAngles || window.jointAngles.length !== q.length) {
                if (window.updateGlobalVariables) {
                    window.updateGlobalVariables();
                }
            }
            
            // 更新关节角度
            for (let i = 0; i < window.jointAngles.length && i < q.length; i++) {
                window.jointAngles[i] = q[i];
            }
            
            // 同步到 main.js 的 jointAngles（确保引用一致）
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            
            // 立即同步更新机器人位姿（避免异步问题）
            if (window.updateRobotPose) {
                window.updateRobotPose();
            }
            
            // 确保拖动控制跟随末端执行器（在拖动过程中也要更新）
            this.updateGizmoPosition();
            
        } catch (error) {
            console.error('更新关节角度失败:', error);
        }
    }
    
    /**
     * 计算矩阵的伪逆（Moore-Penrose伪逆）
     * @param {number[][]} matrix - 输入矩阵
     * @returns {number[][]} 伪逆矩阵
     */
    pseudoInverse(matrix) {
        const rows = matrix.length;
        const cols = matrix[0].length;
        
        // 转置矩阵
        const transpose = [];
        for (let j = 0; j < cols; j++) {
            transpose[j] = [];
            for (let i = 0; i < rows; i++) {
                transpose[j][i] = matrix[i][j];
            }
        }
        
        // 计算 J * J^T
        const jjt = [];
        for (let i = 0; i < rows; i++) {
            jjt[i] = [];
            for (let j = 0; j < rows; j++) {
                let sum = 0;
                for (let k = 0; k < cols; k++) {
                    sum += matrix[i][k] * transpose[k][j];
                }
                jjt[i][j] = sum;
            }
        }
        
        // 添加正则化项（阻尼）
        const lambda = 0.01;
        for (let i = 0; i < rows; i++) {
            jjt[i][i] += lambda;
        }
        
        // 计算 (J * J^T)^(-1)
        // 使用通用矩阵求逆（支持任意大小的方阵）
        const jjtInv = this.inverseMatrix(jjt);
        
        // 计算伪逆: J^+ = J^T * (J * J^T)^(-1)
        const pseudoInv = [];
        for (let i = 0; i < cols; i++) {
            pseudoInv[i] = [];
            for (let j = 0; j < rows; j++) {
                let sum = 0;
                for (let k = 0; k < rows; k++) {
                    sum += transpose[i][k] * jjtInv[k][j];
                }
                pseudoInv[i][j] = sum;
            }
        }
        
        return pseudoInv;
    }
    
    /**
     * 计算通用矩阵的逆（使用高斯-约当消元法）
     * @param {number[][]} matrix - 方阵
     * @returns {number[][]} 逆矩阵
     */
    inverseMatrix(matrix) {
        const n = matrix.length;
        if (matrix[0].length !== n) {
            throw new Error('矩阵必须是方阵');
        }
        
        // 特殊情况：3x3矩阵使用快速公式
        if (n === 3) {
            return this.inverse3x3(matrix);
        }
        
        // 创建增广矩阵 [A | I]
        const augmented = [];
        for (let i = 0; i < n; i++) {
            augmented[i] = [];
            for (let j = 0; j < n; j++) {
                augmented[i][j] = matrix[i][j];
            }
            for (let j = n; j < 2 * n; j++) {
                augmented[i][j] = (j - n === i) ? 1 : 0;
            }
        }
        
        // 高斯-约当消元法
        for (let i = 0; i < n; i++) {
            // 找主元
            let maxRow = i;
            for (let k = i + 1; k < n; k++) {
                if (Math.abs(augmented[k][i]) > Math.abs(augmented[maxRow][i])) {
                    maxRow = k;
                }
            }
            
            // 交换行
            [augmented[i], augmented[maxRow]] = [augmented[maxRow], augmented[i]];
            
            // 检查主元是否为0
            if (Math.abs(augmented[i][i]) < 1e-10) {
                console.warn('矩阵接近奇异，无法求逆');
                // 返回单位矩阵作为fallback
                const identity = [];
                for (let r = 0; r < n; r++) {
                    identity[r] = [];
                    for (let c = 0; c < n; c++) {
                        identity[r][c] = (r === c) ? 1 : 0;
                    }
                }
                return identity;
            }
            
            // 归一化当前行
            const pivot = augmented[i][i];
            for (let j = 0; j < 2 * n; j++) {
                augmented[i][j] /= pivot;
            }
            
            // 消元
            for (let k = 0; k < n; k++) {
                if (k !== i) {
                    const factor = augmented[k][i];
                    for (let j = 0; j < 2 * n; j++) {
                        augmented[k][j] -= factor * augmented[i][j];
                    }
                }
            }
        }
        
        // 提取逆矩阵
        const inverse = [];
        for (let i = 0; i < n; i++) {
            inverse[i] = [];
            for (let j = 0; j < n; j++) {
                inverse[i][j] = augmented[i][j + n];
            }
        }
        
        return inverse;
    }
    
    /**
     * 计算3x3矩阵的逆（快速公式）
     * @param {number[][]} matrix - 3x3矩阵
     * @returns {number[][]} 逆矩阵
     */
    inverse3x3(matrix) {
        const a = matrix[0][0], b = matrix[0][1], c = matrix[0][2];
        const d = matrix[1][0], e = matrix[1][1], f = matrix[1][2];
        const g = matrix[2][0], h = matrix[2][1], i = matrix[2][2];
        
        const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        
        if (Math.abs(det) < 1e-10) {
            console.warn('矩阵行列式接近0，无法求逆');
            return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]; // 返回单位矩阵
        }
        
        const invDet = 1 / det;
        
        return [
            [(e * i - f * h) * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet],
            [(f * g - d * i) * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet],
            [(d * h - e * g) * invDet, (b * g - a * h) * invDet, (a * e - b * d) * invDet]
        ];
    }
}
