/**
 * DQ Robotics JavaScript 接口层
 * 
 * 这个模块封装了 WASM 调用，提供友好的 JavaScript API
 */

let DQModule = null;

/**
 * 初始化 DQ Robotics WASM 模块
 * @returns {Promise<void>}
 */
export async function initDQRobotics() {
    if (DQModule) {
        return DQModule;
    }
    
    try {
        // 使用 script 标签加载 WASM 模块
        // Vite 不允许直接从 /public 导入 JS 文件
        const scriptUrl = '/wasm/dqrobotics.js';
        
        // 检查是否已加载
        if (typeof window.createDQRoboticsModule === 'function') {
            DQModule = await window.createDQRoboticsModule();
            console.log('✓ DQ Robotics WASM 模块加载成功');
            return DQModule;
        }
        
        // 动态添加 script 标签
        const script = document.createElement('script');
        script.src = scriptUrl;
        script.type = 'text/javascript';
        
        await new Promise((resolve, reject) => {
            script.onload = resolve;
            script.onerror = () => reject(new Error(`Failed to load ${scriptUrl}`));
            document.head.appendChild(script);
        });
        
        // 等待模块初始化
        if (typeof window.createDQRoboticsModule !== 'function') {
            throw new Error('createDQRoboticsModule not found after loading script');
        }
        
        DQModule = await window.createDQRoboticsModule();
        console.log('✓ DQ Robotics WASM 模块加载成功');
        return DQModule;
    } catch (error) {
        console.error('加载 DQ Robotics WASM 模块失败:', error);
        throw error;
    }
}

/**
 * 获取DQModule实例（用于直接访问WASM模块）
 * @returns {Object|null}
 */
export function getDQModule() {
    return DQModule;
}

/**
 * DQ 类的 JavaScript 包装
 */
export class DQ {
    constructor(...args) {
        if (!DQModule) {
            throw new Error('DQ Robotics 模块未初始化，请先调用 initDQRobotics()');
        }
        
        if (args.length === 1 && Array.isArray(args[0])) {
            // 从数组创建
            this._dq = DQModule.DQWrapper.createFromArray(args[0]);
        } else if (args.length <= 8) {
            // 从分量创建
            const [q0=0, q1=0, q2=0, q3=0, q4=0, q5=0, q6=0, q7=0] = args;
            this._dq = DQModule.DQWrapper.create(q0, q1, q2, q3, q4, q5, q6, q7);
        } else {
            throw new Error('无效的参数');
        }
    }
    
    /**
     * 获取 DQ 的数组表示
     * @returns {number[]}
     */
    toArray() {
        return DQModule.DQWrapper.toArray(this._dq);
    }
    
    /**
     * DQ 乘法
     * @param {DQ} other
     * @returns {DQ}
     */
    multiply(other) {
        const result = new DQ();
        result._dq = DQModule.DQWrapper.multiply(this._dq, other._dq);
        return result;
    }
    
    /**
     * DQ 加法
     * @param {DQ} other
     * @returns {DQ}
     */
    add(other) {
        const result = new DQ();
        result._dq = DQModule.DQWrapper.add(this._dq, other._dq);
        return result;
    }
    
    /**
     * 共轭
     * @returns {DQ}
     */
    conj() {
        const result = new DQ();
        result._dq = DQModule.DQWrapper.conjugate(this._dq);
        return result;
    }
    
    /**
     * 归一化
     * @returns {DQ}
     */
    normalize() {
        const result = new DQ();
        result._dq = DQModule.DQWrapper.normalize(this._dq);
        return result;
    }
    
    /**
     * 逆
     * @returns {DQ}
     */
    inv() {
        const result = new DQ();
        result._dq = DQModule.DQWrapper.inverse(this._dq);
        return result;
    }
    
    /**
     * 创建旋转 DQ
     * @param {number} angle - 旋转角度（弧度）
     * @param {number} x - 旋转轴 x 分量
     * @param {number} y - 旋转轴 y 分量
     * @param {number} z - 旋转轴 z 分量
     * @returns {DQ}
     */
    static rotation(angle, x, y, z) {
        const result = new DQ();
        result._dq = DQModule.DQWrapper.rotation(angle, x, y, z);
        return result;
    }
    
    /**
     * 创建平移 DQ
     * @param {number} x
     * @param {number} y
     * @param {number} z
     * @returns {DQ}
     */
    static translation(x, y, z) {
        const result = new DQ();
        result._dq = DQModule.DQWrapper.translation(x, y, z);
        return result;
    }
    
    /**
     * 创建位姿 DQ
     * @param {number} rx - 旋转轴 x
     * @param {number} ry - 旋转轴 y
     * @param {number} rz - 旋转轴 z
     * @param {number} angle - 旋转角度
     * @param {number} tx - 平移 x
     * @param {number} ty - 平移 y
     * @param {number} tz - 平移 z
     * @returns {DQ}
     */
    static pose(rx, ry, rz, angle, tx, ty, tz) {
        const result = new DQ();
        result._dq = DQModule.DQWrapper.pose(rx, ry, rz, angle, tx, ty, tz);
        return result;
    }
    
    /**
     * 获取平移部分
     * @returns {number[]}
     */
    getTranslation() {
        return DQModule.DQWrapper.getTranslation(this._dq);
    }
    
    /**
     * 获取旋转部分
     * @returns {number[]}
     */
    getRotation() {
        return DQModule.DQWrapper.getRotation(this._dq);
    }
    
    /**
     * 获取旋转矩阵
     * @returns {number[][]}
     */
    getRotationMatrix() {
        return DQModule.DQWrapper.getRotationMatrix(this._dq);
    }
}

/**
 * 串联机械臂类
 */
export class SerialManipulator {
    constructor(dhMatrix) {
        if (!DQModule) {
            throw new Error('DQ Robotics 模块未初始化');
        }
        this._robot = new DQModule.SerialManipulator(dhMatrix);
    }
    
    /**
     * 正向运动学
     * @param {number[]} jointAngles
     * @returns {number[]}
     */
    fkm(jointAngles) {
        return this._robot.fkm(jointAngles);
    }
    
    /**
     * 获取末端执行器位姿（兼容 FrankaPanda 接口）
     * @param {number[]} jointAngles
     * @returns {{translation: number[], rotation: number[]}}
     */
    getEndEffectorPose(jointAngles) {
        const dqArray = this._robot.fkm(jointAngles);
        // 使用底层 DQ 包装器进行精确分解
        const dqObj = DQModule.DQWrapper.createFromArray(dqArray);
        const translation = DQModule.DQWrapper.getTranslation(dqObj);
        const rotation = DQModule.DQWrapper.getRotation(dqObj);
        return { translation, rotation };
    }

    /**
     * 设置基座平移偏移（Z-up 坐标系）
     * @param {number} x
     * @param {number} y
     * @param {number} z
     */
    setBaseFrame(x, y, z) {
        this._robot.setBaseFrame(x, y, z);
    }

    /**
     * 设置基座完整姿态（旋转+平移），Z-up
     * @param {number} qw - 四元数 w 分量
     * @param {number} qx - 四元数 x 分量
     * @param {number} qy - 四元数 y 分量
     * @param {number} qz - 四元数 z 分量
     * @param {number} tx - 平移 x
     * @param {number} ty - 平移 y
     * @param {number} tz - 平移 z
     */
    setBaseFramePose(qw, qx, qy, qz, tx, ty, tz) {
        this._robot.setBaseFramePose(qw, qx, qy, qz, tx, ty, tz);
    }

    /**
     * 设置基座（直接使用双四元数，8元素数组）
     * @param {number[]} dqArray - 8元素双四元数数组 [qw, qx, qy, qz, qw', qx', qy', qz']
     */
    setBaseFrameDQ(dqArray) {
        if (!Array.isArray(dqArray) || dqArray.length !== 8) {
            throw new Error('双四元数必须是8元素数组');
        }
        this._robot.setBaseFrameDQ(dqArray);
    }

    /**
     * 设置末端平移偏移（相对于末端坐标系，Z-up）
     * @param {number} x
     * @param {number} y
     * @param {number} z
     */
    setEffector(x, y, z) {
        this._robot.setEffector(x, y, z);
    }

    /**
     * 设置末端完整姿态（四元数 + 平移），Z-up
     */
    setEffectorPose(qw, qx, qy, qz, tx, ty, tz) {
        this._robot.setEffectorPose(qw, qx, qy, qz, tx, ty, tz);
    }
    
    /**
     * 位姿雅可比矩阵
     * @param {number[]} jointAngles
     * @returns {number[][]}
     */
    poseJacobian(jointAngles) {
        return this._robot.poseJacobian(jointAngles);
    }
    
    /**
     * 获取配置空间维度（关节数量）
     * @returns {number}
     */
    getDimConfigurationSpace() {
        return this._robot.getDimConfigurationSpace();
    }
}

/**
 * Franka Emika Panda 机器人
 */
export class FrankaPanda {
    constructor() {
        if (!DQModule) {
            throw new Error('DQ Robotics 模块未初始化');
        }
        this._robot = new DQModule.FrankaPanda();
    }
    
    /**
     * 正向运动学
     * @param {number[]} jointAngles - 7 个关节角度（弧度）
     * @returns {number[]} DQ 表示的位姿
     */
    fkm(jointAngles) {
        return this._robot.fkm(jointAngles);
    }
    
    /**
     * 获取末端执行器位姿
     * @param {number[]} jointAngles
     * @returns {{translation: number[], rotation: number[]}}
     */
    getEndEffectorPose(jointAngles) {
        return this._robot.getEndEffectorPose(jointAngles);
    }
    
    /**
     * 位姿雅可比矩阵
     * @param {number[]} jointAngles
     * @returns {number[][]}
     */
    poseJacobian(jointAngles) {
        return this._robot.poseJacobian(jointAngles);
    }
    
    getDimConfigurationSpace() {
        return this._robot.getDimConfigurationSpace();
    }
}

/**
 * Kuka LW4 机器人
 */
export class KukaLw4 {
    constructor() {
        if (!DQModule) {
            throw new Error('DQ Robotics 模块未初始化');
        }
        this._robot = new DQModule.KukaLw4();
    }
    
    fkm(jointAngles) {
        return this._robot.fkm(jointAngles);
    }
    
    getEndEffectorPose(jointAngles) {
        return this._robot.getEndEffectorPose(jointAngles);
    }
    
    getDimConfigurationSpace() {
        return this._robot.getDimConfigurationSpace();
    }
}

/**
 * 工具函数：将 DQ 四元数转换为旋转矩阵
 * @param {number[]} quaternion - [w, x, y, z]
 * @returns {number[][]} 3x3 旋转矩阵
 */
export function quaternionToRotationMatrix(quaternion) {
    const [w, x, y, z] = quaternion.slice(0, 4);
    
    const xx = x * x;
    const yy = y * y;
    const zz = z * z;
    const xy = x * y;
    const xz = x * z;
    const yz = y * z;
    const wx = w * x;
    const wy = w * y;
    const wz = w * z;
    
    return [
        [1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy)],
        [2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)]
    ];
}

/**
 * 工具函数：将旋转矩阵转换为欧拉角（ZYX 顺序）
 * @param {number[][]} matrix
 * @returns {{roll: number, pitch: number, yaw: number}}
 */
export function rotationMatrixToEuler(matrix) {
    const sy = Math.sqrt(matrix[0][0] * matrix[0][0] + matrix[1][0] * matrix[1][0]);
    
    const singular = sy < 1e-6;
    
    let roll, pitch, yaw;
    
    if (!singular) {
        roll = Math.atan2(matrix[2][1], matrix[2][2]);
        pitch = Math.atan2(-matrix[2][0], sy);
        yaw = Math.atan2(matrix[1][0], matrix[0][0]);
    } else {
        roll = Math.atan2(-matrix[1][2], matrix[1][1]);
        pitch = Math.atan2(-matrix[2][0], sy);
        yaw = 0;
    }
    
    return { roll, pitch, yaw };
}

/**
 * 协作双臂系统类
 */
export class CooperativeDualArm {
    constructor(robot1DH, robot2DH) {
        if (!DQModule) {
            throw new Error('DQ Robotics 模块未初始化');
        }
        this._dualArm = new DQModule.CooperativeDualTaskSpace(robot1DH, robot2DH);
        this.robot1JointCount = robot1DH[0].length;
        this.robot2JointCount = robot2DH[0].length;
        this.totalJointCount = this.robot1JointCount + this.robot2JointCount;
    }
    
    /**
     * 设置机器人1的基座位姿（相对于世界坐标系原点，Z-up）
     * @param {number} x
     * @param {number} y
     * @param {number} z
     */
    setRobot1Base(x, y, z) {
        this._dualArm.setRobot1Base(x, y, z);
    }
    
    /**
     * 设置机器人1的基座位姿（完整姿态：旋转+平移）
     */
    setRobot1BasePose(qw, qx, qy, qz, tx, ty, tz) {
        this._dualArm.setRobot1BasePose(qw, qx, qy, qz, tx, ty, tz);
    }
    
    /**
     * 设置机器人1的基座（直接使用双四元数，8元素数组）
     * @param {number[]} dqArray - 8元素双四元数数组 [qw, qx, qy, qz, qw', qx', qy', qz']
     */
    setRobot1BaseDQ(dqArray) {
        if (!Array.isArray(dqArray) || dqArray.length !== 8) {
            throw new Error('双四元数必须是8元素数组');
        }
        this._dualArm.setRobot1BaseDQ(dqArray);
    }
    
    /**
     * 设置机器人1的末端执行器偏移（仅平移）
     */
    setRobot1Effector(x, y, z) {
        this._dualArm.setRobot1Effector(x, y, z);
    }
    
    /**
     * 设置机器人1的末端执行器偏移（完整姿态）
     */
    setRobot1EffectorPose(qw, qx, qy, qz, tx, ty, tz) {
        this._dualArm.setRobot1EffectorPose(qw, qx, qy, qz, tx, ty, tz);
    }
    
    /**
     * 设置机器人2的基座位姿（仅平移）
     */
    setRobot2Base(x, y, z) {
        this._dualArm.setRobot2Base(x, y, z);
    }
    
    /**
     * 设置机器人2的基座位姿（完整姿态：旋转+平移）
     */
    setRobot2BasePose(qw, qx, qy, qz, tx, ty, tz) {
        this._dualArm.setRobot2BasePose(qw, qx, qy, qz, tx, ty, tz);
    }
    
    /**
     * 设置机器人2的基座（直接使用双四元数，8元素数组）
     * @param {number[]} dqArray - 8元素双四元数数组 [qw, qx, qy, qz, qw', qx', qy', qz']
     */
    setRobot2BaseDQ(dqArray) {
        if (!Array.isArray(dqArray) || dqArray.length !== 8) {
            throw new Error('双四元数必须是8元素数组');
        }
        this._dualArm.setRobot2BaseDQ(dqArray);
    }
    
    /**
     * 设置机器人2的末端执行器偏移（仅平移）
     */
    setRobot2Effector(x, y, z) {
        this._dualArm.setRobot2Effector(x, y, z);
    }
    
    /**
     * 设置机器人2的末端执行器偏移（完整姿态）
     */
    setRobot2EffectorPose(qw, qx, qy, qz, tx, ty, tz) {
        this._dualArm.setRobot2EffectorPose(qw, qx, qy, qz, tx, ty, tz);
    }
    
    /**
     * 获取机器人1的位姿（相对于其基座）
     * @param {number[]} jointAngles - 所有关节角度 [robot1_joints..., robot2_joints...]
     * @returns {number[]} DQ数组（8元素）
     */
    getRobot1Pose(jointAngles) {
        return this._dualArm.pose1(jointAngles);
    }
    
    /**
     * 获取机器人2的位姿（相对于其基座）
     * @param {number[]} jointAngles
     * @returns {number[]} DQ数组（8元素）
     */
    getRobot2Pose(jointAngles) {
        return this._dualArm.pose2(jointAngles);
    }
    
    /**
     * 获取相对位姿（robot2相对于robot1的变换）
     * relative_pose = pose2 * pose1^(-1)
     * @param {number[]} jointAngles
     * @returns {number[]} DQ数组（8元素）
     */
    getRelativePose(jointAngles) {
        return this._dualArm.relativePose(jointAngles);
    }
    
    /**
     * 获取绝对位姿（两个末端执行器的平均位置）
     * absolute_pose = (pose1 + pose2) / 2
     * @param {number[]} jointAngles
     * @returns {number[]} DQ数组（8元素）
     */
    getAbsolutePose(jointAngles) {
        return this._dualArm.absolutePose(jointAngles);
    }
    
    /**
     * 获取机器人1相对于世界坐标系原点的位姿
     * @param {number[]} jointAngles
     * @returns {{translation: number[], rotation: number[]}}
     */
    getRobot1WorldPose(jointAngles) {
        const dqArray = this.getRobot1Pose(jointAngles);
        const dqObj = DQModule.DQWrapper.createFromArray(dqArray);
        const translation = DQModule.DQWrapper.getTranslation(dqObj);
        const rotation = DQModule.DQWrapper.getRotation(dqObj);
        return { translation, rotation };
    }
    
    /**
     * 获取机器人2相对于世界坐标系原点的位姿
     * @param {number[]} jointAngles
     * @returns {{translation: number[], rotation: number[]}}
     */
    getRobot2WorldPose(jointAngles) {
        const dqArray = this.getRobot2Pose(jointAngles);
        const dqObj = DQModule.DQWrapper.createFromArray(dqArray);
        const translation = DQModule.DQWrapper.getTranslation(dqObj);
        const rotation = DQModule.DQWrapper.getRotation(dqObj);
        return { translation, rotation };
    }
    
    /**
     * 获取绝对位姿（相对于世界坐标系原点）
     * @param {number[]} jointAngles
     * @returns {{translation: number[], rotation: number[]}}
     */
    getAbsoluteWorldPose(jointAngles) {
        const dqArray = this.getAbsolutePose(jointAngles);
        const dqObj = DQModule.DQWrapper.createFromArray(dqArray);
        const translation = DQModule.DQWrapper.getTranslation(dqObj);
        const rotation = DQModule.DQWrapper.getRotation(dqObj);
        return { translation, rotation };
    }
    
    /**
     * 获取相对位姿（结构化的）
     * @param {number[]} jointAngles
     * @returns {{translation: number[], rotation: number[]}}
     */
    getRelativePoseStructured(jointAngles) {
        const dqArray = this.getRelativePose(jointAngles);
        const dqObj = DQModule.DQWrapper.createFromArray(dqArray);
        const translation = DQModule.DQWrapper.getTranslation(dqObj);
        const rotation = DQModule.DQWrapper.getRotation(dqObj);
        return { translation, rotation };
    }
    
    /**
     * 获取相对位姿雅可比矩阵
     * @param {number[]} jointAngles
     * @returns {number[][]}
     */
    getRelativePoseJacobian(jointAngles) {
        return this._dualArm.relativePoseJacobian(jointAngles);
    }
    
    /**
     * 获取绝对位姿雅可比矩阵
     * @param {number[]} jointAngles
     * @returns {number[][]}
     */
    getAbsolutePoseJacobian(jointAngles) {
        return this._dualArm.absolutePoseJacobian(jointAngles);
    }
}

