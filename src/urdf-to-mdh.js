/**
 * URDF 到 Modified DH 参数转换器
 * 
 * 参考: https://github.com/Democratizing-Dexterous/URDFly
 * 
 * 将 URDF 关节链转换为 Modified DH 参数，用于 DQ Robotics 运动学计算
 */

import * as THREE from 'three';

const WORLD_Z = new THREE.Vector3(0, 0, 1);
const WORLD_Y = new THREE.Vector3(0, 1, 0);
const Y_ALIGNMENT_THRESHOLD = 0.99; // treat as Y-up only when strongly aligned

function shouldRotateYUpToZUp(zAxis) {
    if (!zAxis) {
        return false;
    }
    const normalized = zAxis.clone().normalize();
    const dotY = Math.abs(normalized.dot(WORLD_Y));
    const dotZ = Math.abs(normalized.dot(WORLD_Z));
    return dotY > Y_ALIGNMENT_THRESHOLD && dotY > dotZ;
}

function buildPoseFromAxes(origin, xAxis, yAxis, zAxis, logLabel = '') {
    if (!origin || !xAxis || !yAxis || !zAxis) {
        return null;
    }

    const orthoX = xAxis.clone().normalize();
    const orthoY = yAxis.clone().normalize();
    const orthoZ = zAxis.clone().normalize();

    let rotationMatrix = new THREE.Matrix4().makeBasis(orthoX, orthoY, orthoZ);
    let quaternion = new THREE.Quaternion().setFromRotationMatrix(rotationMatrix).normalize();
    let position = origin.clone();

    if (shouldRotateYUpToZUp(orthoZ)) {
        const yToZTransform = new THREE.Matrix4().makeRotationX(Math.PI / 2);
        rotationMatrix = new THREE.Matrix4().multiplyMatrices(yToZTransform, rotationMatrix);
        quaternion = new THREE.Quaternion().setFromRotationMatrix(rotationMatrix).normalize();
        position = position.applyMatrix4(yToZTransform);
        if (logLabel) {
            console.log(`${logLabel} [Y->Z] 侦测到Y-up坐标系，已旋转至Z-up`);
        }
    } else if (logLabel) {
        console.log(`${logLabel} [Z] 坐标系已为Z-up，无需转换`);
    }

    return {
        quaternion,
        position
    };
}

/**
 * 从 URDF 机器人模型提取 Modified DH 参数
 * 
 * 参考 URDFly 和 DQ Robotics 的实现
 * 
 * @param {Object} urdfRobot - urdf-loader 加载的机器人对象
 * @param {Array<string>} jointNames - 按顺序排列的关节名称
 * @param {boolean} convertToZUp - 是否转换到 Z-up 坐标系（默认 false，保持URDF的Y-up）
 * @param {string} endLinkName - 可选的末端执行器链接名称（用于提取fixed joint偏移）
 * @param {Object} baseConfig - 可选的基座配置 { origin: [x, y, z], z_axis: [x, y, z], x_axis: [x, y, z] }
 * @param {string|null} baseLinkName - 可选的基座链接名称，用于自动估计基座位姿
 * @returns {Object} { mdh: MDH参数矩阵, effectorPose: 末端执行器位姿（DQ格式）, autoBasePose: 自动估计的基座位姿（若存在） }
 */
export function extractMDHFromURDF(urdfRobot, jointNames, convertToZUp = false, endLinkName = null, baseConfig = null, baseLinkName = null) {
    if (!urdfRobot || !jointNames || jointNames.length === 0) {
        throw new Error('Invalid URDF robot or joint names');
    }
    
    console.log(`\n开始从 URDF 提取 MDH 参数...`);
    if (typeof urdfRobot.updateMatrixWorld === 'function') {
        urdfRobot.updateMatrixWorld(true);
    }
    
    // 收集每个关节的变换信息
    const jointData = [];
    let autoBasePose = null;
    
    for (let i = 0; i < jointNames.length; i++) {
        const jointName = jointNames[i];
        const joint = urdfRobot.joints[jointName];
        
        if (!joint) {
            console.warn(`关节 ${jointName} 未找到`);
            continue;
        }
        
        // 确保变换矩阵是最新的
        joint.updateMatrixWorld(true);
        
        // 优先使用URDF loader直接提供的position和quaternion（相对于parent）
        // 这些值直接从URDF的<origin>标签解析，更可靠
        let position, quaternion;
        if (joint.position && joint.quaternion) {
            // URDF loader已经解析了origin，直接使用
            position = joint.position.clone();
            quaternion = joint.quaternion.clone();
            console.log(`  ✓ 使用URDF origin数据（直接解析）`);
        } else {
            // 回退：从矩阵分解
        const localMatrix = joint.matrix.clone();
            position = new THREE.Vector3();
            quaternion = new THREE.Quaternion();
        const scale = new THREE.Vector3();
        localMatrix.decompose(position, quaternion, scale);
            console.log(`  ⚠ 使用矩阵分解数据`);
        }
        
        // 获取关节轴（在局部坐标系中）
        const axis = joint.axis ? joint.axis.clone() : new THREE.Vector3(0, 0, 1);
        
        // 将欧拉角从四元数提取
        const euler = new THREE.Euler().setFromQuaternion(quaternion, 'XYZ');
        
        // 从position和quaternion重建变换矩阵（用于后续计算）
        const localMatrix = new THREE.Matrix4();
        localMatrix.compose(position, quaternion, new THREE.Vector3(1, 1, 1));
        
        jointData.push({
            name: jointName,
            index: i,
            position: position,
            quaternion: quaternion,
            euler: euler,
            axis: axis,
            matrix: localMatrix  // 重建的局部变换矩阵
        });
        
        console.log(`  关节 ${i+1} (${jointName}):
            xyz = [${position.x.toFixed(4)}, ${position.y.toFixed(4)}, ${position.z.toFixed(4)}]
            rpy = [${euler.x.toFixed(4)}, ${euler.y.toFixed(4)}, ${euler.z.toFixed(4)}]
            axis = [${axis.x.toFixed(1)}, ${axis.y.toFixed(1)}, ${axis.z.toFixed(1)}]`);
    }
    
    // 注意：MDH参数应该只计算到最后一个旋转关节
    // fixed joint（如rjoint8/ljoint8）的偏移应该通过setEffectorPose单独设置，而不是包含在MDH中
    // 这里不再查找fixed joint的位置，MDH只计算旋转关节链
    
    // 从关节变换提取MDH参数（不包含fixed joint偏移）
    // 同时返回MDH坐标系信息，用于坐标系转换
    // baseConfig: { origin: [x, y, z], z_axis: [x, y, z], x_axis: [x, y, z] } 或 null（使用默认值）
    const { mdhParams, mdhFrameInfo, allMDHFrames, baseFrameInfo } = computeMDHFromJointChain(jointData, null, baseConfig);
    
    console.log('\n  提取的 MDH 参数:');
    console.log('  关节 |    θ     |    d     |    a     |    α    ');
    console.log('  -----|----------|----------|----------|----------');
    mdhParams.forEach((params, i) => {
        console.log(`    ${(i+1).toString().padStart(2)}  | ` +
                   `${params[0].toFixed(4).padStart(8)} | ` +
                   `${params[1].toFixed(4).padStart(8)} | ` +
                   `${params[2].toFixed(4).padStart(8)} | ` +
                   `${params[3].toFixed(4).padStart(8)}`);
    });

    const buildPoseFromMDHFrame = (frame) => {
        if (!frame) return null;
        return buildPoseFromAxes(frame.origin, frame.xAxis, frame.yAxis, frame.zAxis, '  ⎈ 基座自动估计');
    };

    if (!autoBasePose && baseFrameInfo?.firstJointFrame) {
        const poseInfo = buildPoseFromMDHFrame(baseFrameInfo.firstJointFrame);
        if (poseInfo) {
            autoBasePose = [
                poseInfo.quaternion.w,
                poseInfo.quaternion.x,
                poseInfo.quaternion.y,
                poseInfo.quaternion.z,
                poseInfo.position.x,
                poseInfo.position.y,
                poseInfo.position.z
            ];
            console.log('  ⎈ 基于首个 MDH 坐标系自动估计基座位姿');
            console.log(`    平移(Z-up): (${poseInfo.position.x.toFixed(4)}, ${poseInfo.position.y.toFixed(4)}, ${poseInfo.position.z.toFixed(4)})`);
            console.log(`    四元数(Z-up): (${poseInfo.quaternion.w.toFixed(4)}, ${poseInfo.quaternion.x.toFixed(4)}, ${poseInfo.quaternion.y.toFixed(4)}, ${poseInfo.quaternion.z.toFixed(4)})`);
        }
    }

    if (!autoBasePose && baseLinkName && urdfRobot.links && urdfRobot.links[baseLinkName]) {
        try {
            const baseLink = urdfRobot.links[baseLinkName];
            if (typeof baseLink.updateMatrixWorld === 'function') {
                baseLink.updateMatrixWorld(true);
            }
            if (baseLink.matrixWorld) {
                const baseMatrix = baseLink.matrixWorld.clone();
                const baseOrigin = new THREE.Vector3().setFromMatrixPosition(baseMatrix);
                const baseRotation = new THREE.Matrix4().extractRotation(baseMatrix);
                const xAxis = new THREE.Vector3();
                const yAxis = new THREE.Vector3();
                const zAxis = new THREE.Vector3();
                baseRotation.extractBasis(xAxis, yAxis, zAxis);

                const poseInfo = buildPoseFromAxes(baseOrigin, xAxis, yAxis, zAxis, '  ⎈ 回退基座');
                if (poseInfo) {
                    autoBasePose = [
                        poseInfo.quaternion.w, poseInfo.quaternion.x, poseInfo.quaternion.y, poseInfo.quaternion.z,
                        poseInfo.position.x, poseInfo.position.y, poseInfo.position.z
                    ];
                    console.log('  ⎈ 回退：基于 base_link 自动估计基座位姿');
                    console.log(`    基座链接: ${baseLinkName}`);
                    console.log(`    平移: (${poseInfo.position.x.toFixed(4)}, ${poseInfo.position.y.toFixed(4)}, ${poseInfo.position.z.toFixed(4)})`);
                    console.log(`    四元数: (${poseInfo.quaternion.w.toFixed(4)}, ${poseInfo.quaternion.x.toFixed(4)}, ${poseInfo.quaternion.y.toFixed(4)}, ${poseInfo.quaternion.z.toFixed(4)})`);
                }
            }
        } catch (error) {
            console.warn(`  ⚠ 回退估计基座位姿失败 (${baseLinkName}):`, error.message);
        }
    }
    
    // 提取fixed joint偏移作为末端执行器位姿
    // 新方法：通过正向运动学计算末端执行器和最后一个MDH关节的位姿，然后计算相对变换
    let effectorPose = null;
    if (endLinkName && urdfRobot) {
        effectorPose = computeEffectorOffsetFromForwardKinematics(urdfRobot, jointNames, jointData, endLinkName, mdhFrameInfo);
    } else if (jointNames.length > 0 && urdfRobot) {
        // 如果没有指定endLinkName，尝试自动查找最后一个旋转关节的child link后的fixed joint
        const lastJointName = jointNames[jointNames.length - 1];
        const lastJoint = urdfRobot.joints[lastJointName];
        if (lastJoint) {
            // 获取最后一个旋转关节的child link
            let lastJointChild = null;
            if (typeof lastJoint.child === 'string') {
                lastJointChild = lastJoint.child;
            } else if (lastJoint.childLink) {
                lastJointChild = lastJoint.childLink;
            } else if (lastJoint.child && typeof lastJoint.child === 'object' && lastJoint.child.name) {
                lastJointChild = lastJoint.child.name;
            }
            
            if (lastJointChild) {
                console.log(`\n  尝试自动查找从 ${lastJointChild} 到末端执行器的fixed joint...`);
                // 查找fixed joint
                const fixedJoint = findFixedJointFromParentLink(urdfRobot, lastJointChild, null);
                if (fixedJoint) {
                    // 通过fixed joint找到endLinkName
                    let endLink = null;
                    for (const name of Object.keys(urdfRobot.joints)) {
                        const j = urdfRobot.joints[name];
                        const isFixed = (j.jointType || j.type) === 'fixed';
                        if (isFixed) {
                            // 尝试获取child
                            let childName = null;
                            if (typeof j.child === 'string') {
                                childName = j.child;
                            } else if (j.childLink) {
                                childName = j.childLink;
                            } else if (j.child && typeof j.child === 'object' && j.child.name) {
                                childName = j.child.name;
                            }
                            if (childName) {
                                endLink = childName;
                                break;
                            }
                        }
                    }
                    if (endLink) {
                        effectorPose = computeEffectorOffsetFromForwardKinematics(urdfRobot, jointNames, jointData, endLink, mdhFrameInfo);
                    }
                }
            }
        }
    }
    
    // 返回MDH参数和所有MDH坐标系信息（用于可视化）
    return {
        mdh: mdhParams,
        effectorPose: effectorPose,
        allMDHFrames: allMDHFrames,  // 所有MDH坐标系信息（用于可视化）
        autoBasePose: autoBasePose,
        baseFrameInfo: baseFrameInfo
    };
}

/**
 * 通过正向运动学计算末端执行器偏移
 * 方法：计算末端执行器和最后一个MDH关节在世界坐标系中的位姿，然后计算相对变换
 * 
 * @param {Object} urdfRobot - URDF机器人对象
 * @param {Array<string>} jointNames - 关节名称列表
 * @param {Array} jointData - 关节数据数组
 * @param {string} endLinkName - 末端执行器链接名称
 * @param {Object} mdhFrameInfo - MDH最后一个关节的坐标系信息
 * @returns {Object|null} { qw, qx, qy, qz, tx, ty, tz } 或 null
 */
function computeEffectorOffsetFromForwardKinematics(urdfRobot, jointNames, jointData, endLinkName, mdhFrameInfo) {
    console.log(`\n  方法：计算末端执行器偏移（新思路）...`);
    console.log(`    目标末端链接: ${endLinkName}`);
    
    // 步骤1: 找到最后一个旋转关节（固定偏移前的关节）
    if (jointNames.length === 0) {
        console.warn(`    ⚠ 没有旋转关节`);
        return null;
    }
    
    const lastRotJointName = jointNames[jointNames.length - 1];
    const lastRotJoint = urdfRobot.joints[lastRotJointName];
    if (!lastRotJoint) {
        console.warn(`    ⚠ 未找到最后一个旋转关节: ${lastRotJointName}`);
        return null;
    }
    
    // 获取最后一个旋转关节的child link（这是固定偏移前的关节）
    // 使用与extractFixedJointOffset相同的健壮逻辑
    let lastRotJointChildName = null;
    
    // 方法1: 直接字符串属性
    if (typeof lastRotJoint.child === 'string') {
        lastRotJointChildName = lastRotJoint.child;
    } else if (lastRotJoint.childLink && typeof lastRotJoint.childLink === 'string') {
        lastRotJointChildName = lastRotJoint.childLink;
    }
    // 方法2: 对象属性
    else if (lastRotJoint.child && typeof lastRotJoint.child === 'object') {
        lastRotJointChildName = lastRotJoint.child.name || lastRotJoint.child.link || lastRotJoint.child.linkName || null;
        
        // 如果还是null，可能是URDF loader的THREE.Object3D，尝试通过urdfRobot.links反向查找
        if (!lastRotJointChildName && urdfRobot.links) {
            const linkNames = Object.keys(urdfRobot.links);
            const matchingLink = linkNames.find(linkName => {
                const linkObj = urdfRobot.links[linkName];
                return linkObj === lastRotJoint.child || (linkObj && linkObj.uuid === lastRotJoint.child?.uuid);
            });
            if (matchingLink) {
                lastRotJointChildName = matchingLink;
            }
        }
    }
    
    // 方法3: 如果childName仍然是null，尝试通过urdfRobot.links反向查找
    if (!lastRotJointChildName && urdfRobot.links && lastRotJoint.child) {
        const linkNames = Object.keys(urdfRobot.links);
        for (const linkName of linkNames) {
            const linkObj = urdfRobot.links[linkName];
            if (linkObj === lastRotJoint.child || (linkObj && lastRotJoint.child && linkObj.uuid === lastRotJoint.child.uuid)) {
                lastRotJointChildName = linkName;
                break;
            }
        }
    }
    
    // 方法4: 尝试从joint的children属性查找（THREE.js场景图）
    if (!lastRotJointChildName && lastRotJoint.children && Array.isArray(lastRotJoint.children) && lastRotJoint.children.length > 0) {
        for (const childObj of lastRotJoint.children) {
            if (childObj && urdfRobot.links) {
                const linkNames = Object.keys(urdfRobot.links);
                for (const linkName of linkNames) {
                    if (urdfRobot.links[linkName] === childObj) {
                        lastRotJointChildName = linkName;
                        break;
                    }
                }
                if (lastRotJointChildName) break;
            }
        }
    }
    
    if (!lastRotJointChildName) {
        console.warn(`    ⚠ 无法找到最后一个旋转关节的child link`);
        console.warn(`    调试信息: joint =`, lastRotJoint);
        console.warn(`    joint.child =`, lastRotJoint.child);
        console.warn(`    joint.childLink =`, lastRotJoint.childLink);
        console.warn(`    joint.children =`, lastRotJoint.children);
        
        // 如果找不到child link，尝试直接使用endLinkName（如果没有fixed joint的情况）
        // 这种情况下，最后一个旋转关节可能直接连接到end link
        if (endLinkName) {
            console.log(`    回退：假设最后一个旋转关节直接连接到 ${endLinkName}`);
            lastRotJointChildName = endLinkName;
        } else {
            return null;
        }
    }
    
    console.log(`    最后一个旋转关节: ${lastRotJointName}`);
    console.log(`    固定偏移前的link: ${lastRotJointChildName}`);
    
    // 步骤2: 获取固定偏移前的link在URDF中的位姿（Y-up）
    const lastRotLink = urdfRobot.links[lastRotJointChildName];
    if (!lastRotLink) {
        console.warn(`    ⚠ 未找到link: ${lastRotJointChildName}`);
        return null;
    }
    
    lastRotLink.updateMatrixWorld(true);
    const lastRotLinkMatrixY = lastRotLink.matrixWorld.clone();
    const lastRotLinkPosY = new THREE.Vector3().setFromMatrixPosition(lastRotLinkMatrixY);
    const lastRotLinkRotY = new THREE.Matrix4().extractRotation(lastRotLinkMatrixY);
    const lastRotLinkQuatY = new THREE.Quaternion().setFromRotationMatrix(lastRotLinkRotY);
    
    console.log(`    固定偏移前的link位姿 (Y-up):`);
    console.log(`      位置: (${lastRotLinkPosY.x.toFixed(4)}, ${lastRotLinkPosY.y.toFixed(4)}, ${lastRotLinkPosY.z.toFixed(4)})`);
    console.log(`      四元数: (${lastRotLinkQuatY.w.toFixed(4)}, ${lastRotLinkQuatY.x.toFixed(4)}, ${lastRotLinkQuatY.y.toFixed(4)}, ${lastRotLinkQuatY.z.toFixed(4)})`);
    
    // 步骤3: 找到fixed joint并获取其偏移（在URDF坐标系中，相对于parent link）
    // 首先检查：如果lastRotJointChildName已经等于endLinkName，说明没有fixed joint
    if (lastRotJointChildName === endLinkName) {
        console.log(`    ⊘ 最后一个旋转关节直接连接到末端link（${endLinkName}），没有fixed joint，末端偏移为0`);
        return null;
    }
    
    const fixedJoint = findFixedJointFromParentLink(urdfRobot, lastRotJointChildName, mdhFrameInfo, endLinkName);
    if (!fixedJoint) {
        console.log(`    ⊘ 未找到fixed joint（从${lastRotJointChildName}到${endLinkName}），末端偏移为0`);
        return null;
    }
    
    // 尝试获取fixed joint的名称（用于日志）
    // 注意：joint对象可能没有name属性，我们通过调试信息获取
    let fixedJointName = 'fixed_joint';
    // 尝试从joint对象获取名称
    if (fixedJoint.name) {
        fixedJointName = fixedJoint.name;
    } else {
        // 通过urdfRobot.joints反向查找
        for (const name of Object.keys(urdfRobot.joints || {})) {
            if (urdfRobot.joints[name] === fixedJoint) {
                fixedJointName = name;
                break;
            }
        }
    }
    console.log(`    找到fixed joint: ${fixedJointName}`);
    
    // 获取fixed joint的origin（在parent link坐标系中）
    // URDFLoader会将origin解析为position和quaternion，就像旋转关节一样
    let fixedPosY, fixedQuatY;
    
    if (fixedJoint.position && fixedJoint.quaternion) {
        // URDFLoader已经解析了origin，直接使用
        fixedPosY = fixedJoint.position.clone();
        fixedQuatY = fixedJoint.quaternion.clone();
        console.log(`    ✓ 使用URDFLoader解析的position和quaternion`);
    } else {
        // 回退：尝试从origin对象获取
        const fixedOrigin = fixedJoint.origin || fixedJoint.joint?.origin;
        if (!fixedOrigin) {
            console.warn(`    ⚠ fixed joint没有origin、position或quaternion`);
            console.warn(`    调试信息: fixedJoint =`, fixedJoint);
            return null;
        }
        
        fixedPosY = new THREE.Vector3(
            fixedOrigin.xyz?.[0] || fixedOrigin.x || 0,
            fixedOrigin.xyz?.[1] || fixedOrigin.y || 0,
            fixedOrigin.xyz?.[2] || fixedOrigin.z || 0
        );
        
        let fixedRotY = new THREE.Matrix4();
        if (fixedOrigin.rpy) {
            const rpy = fixedOrigin.rpy;
            fixedRotY.makeRotationFromEuler(new THREE.Euler(rpy[0], rpy[1], rpy[2], 'XYZ'));
        } else if (fixedOrigin.roll !== undefined || fixedOrigin.pitch !== undefined || fixedOrigin.yaw !== undefined) {
            fixedRotY.makeRotationFromEuler(new THREE.Euler(
                fixedOrigin.roll || 0,
                fixedOrigin.pitch || 0,
                fixedOrigin.yaw || 0,
                'XYZ'
            ));
        }
        fixedQuatY = new THREE.Quaternion().setFromRotationMatrix(fixedRotY);
        console.log(`    ⚠ 使用origin对象解析`);
    }
    
    console.log(`    Fixed joint偏移 (在parent link坐标系，Y-up):`);
    console.log(`      位置: (${fixedPosY.x.toFixed(4)}, ${fixedPosY.y.toFixed(4)}, ${fixedPosY.z.toFixed(4)})`);
    console.log(`      四元数: (${fixedQuatY.w.toFixed(4)}, ${fixedQuatY.x.toFixed(4)}, ${fixedQuatY.y.toFixed(4)}, ${fixedQuatY.z.toFixed(4)})`);
    
    // 步骤4: 获取MDH最后一个关节的位姿（Z-up）
    const yToZTransform = new THREE.Matrix4().makeRotationX(Math.PI / 2);
    
    const mdhLastOriginY = mdhFrameInfo.lastOrigin;
    const mdhRotationY = new THREE.Matrix4();
    mdhRotationY.makeBasis(mdhFrameInfo.lastX, mdhFrameInfo.lastY, mdhFrameInfo.lastZ);
    
    // MDH关节在Y-up中的变换矩阵
    const mdhToWorldY = new THREE.Matrix4();
    mdhToWorldY.compose(mdhLastOriginY, new THREE.Quaternion().setFromRotationMatrix(mdhRotationY), new THREE.Vector3(1, 1, 1));
    
    // 转换为Z-up
    const mdhOriginYVec = new THREE.Vector3().setFromMatrixPosition(mdhToWorldY);
    const mdhOriginZVec = mdhOriginYVec.clone().applyMatrix4(yToZTransform);
    
    const mdhRotY = new THREE.Matrix4().extractRotation(mdhToWorldY);
    const mdhRotZ = new THREE.Matrix4();
    mdhRotZ.multiplyMatrices(yToZTransform, mdhRotY);
    const mdhRotQuat = new THREE.Quaternion().setFromRotationMatrix(mdhRotZ);
    
    console.log(`    MDH最后一个关节位姿 (Z-up):`);
    console.log(`      位置: (${mdhOriginZVec.x.toFixed(4)}, ${mdhOriginZVec.y.toFixed(4)}, ${mdhOriginZVec.z.toFixed(4)})`);
    console.log(`      四元数: (${mdhRotQuat.w.toFixed(4)}, ${mdhRotQuat.x.toFixed(4)}, ${mdhRotQuat.y.toFixed(4)}, ${mdhRotQuat.z.toFixed(4)})`);
    
    // 步骤5: 将fixed joint的偏移从parent link坐标系（Y-up）转换到MDH坐标系（Z-up）
    // 5.1 先将fixed偏移从parent link坐标系转换到世界坐标系（Y-up）
    // fixed偏移在parent link坐标系中表示为fixedPosY和fixedQuatY
    // 需要将其转换到世界坐标系
    const fixedPosInWorldY = fixedPosY.clone().applyQuaternion(lastRotLinkQuatY);
    
    // 5.2 将世界坐标系中的偏移从Y-up转换到Z-up
    const fixedPosInWorldZ = fixedPosInWorldY.clone().applyMatrix4(yToZTransform);
    
    console.log(`    Fixed偏移转换到世界坐标系 (Z-up): (${fixedPosInWorldZ.x.toFixed(4)}, ${fixedPosInWorldZ.y.toFixed(4)}, ${fixedPosInWorldZ.z.toFixed(4)})`);
    
    // 5.3 将偏移从Z-up世界坐标系转换到MDH坐标系（本地坐标系）
    const mdhRotInverse = mdhRotQuat.clone().invert();
    const offsetInMDHFrame = fixedPosInWorldZ.clone().applyQuaternion(mdhRotInverse);
    
    console.log(`    Fixed偏移在MDH坐标系中: (${offsetInMDHFrame.x.toFixed(4)}, ${offsetInMDHFrame.y.toFixed(4)}, ${offsetInMDHFrame.z.toFixed(4)})`);
    
    // 步骤6: 构建对偶四元数
    // 旋转部分：将fixed joint的旋转也转换到MDH坐标系
    const fixedRotYMatrix = new THREE.Matrix4().makeRotationFromQuaternion(fixedQuatY);
    const fixedRotZMatrix = new THREE.Matrix4();
    fixedRotZMatrix.multiplyMatrices(yToZTransform, fixedRotYMatrix);
    const fixedRotZQuat = new THREE.Quaternion().setFromRotationMatrix(fixedRotZMatrix);
    
    // 相对旋转：fixed joint的旋转相对于MDH坐标系的旋转
    const offsetRot = fixedRotZQuat.clone().premultiply(mdhRotInverse);
    
    console.log(`    最终偏移 (MDH坐标系):`);
    console.log(`      位置: (${offsetInMDHFrame.x.toFixed(4)}, ${offsetInMDHFrame.y.toFixed(4)}, ${offsetInMDHFrame.z.toFixed(4)})`);
    console.log(`      四元数: (${offsetRot.w.toFixed(4)}, ${offsetRot.x.toFixed(4)}, ${offsetRot.y.toFixed(4)}, ${offsetRot.z.toFixed(4)})`);
    
    return {
        qw: offsetRot.w,
        qx: offsetRot.x,
        qy: offsetRot.y,
        qz: offsetRot.z,
        tx: offsetInMDHFrame.x,
        ty: offsetInMDHFrame.y,
        tz: offsetInMDHFrame.z
    };
}

/**
 * 提取fixed joint偏移作为末端执行器位姿（旧方法，保留作为备用）
 * 
 * 注意：fixed joint的偏移在URDF中是相对于其parent link的。
 * setEffectorPose期望的偏移应该是相对于最后一个关节的MDH坐标系的。
 * 需要将fixed joint的偏移从parent link坐标系转换到MDH最后一个关节坐标系。
 * 
 * @param {Object} urdfRobot - URDF机器人对象
 * @param {Array<string>} jointNames - 关节名称列表
 * @param {string} endLinkName - 末端执行器链接名称
 * @param {Object} mdhFrameInfo - MDH最后一个关节的坐标系信息 {lastOrigin, lastX, lastY, lastZ}
 * @returns {Object|null} { qw, qx, qy, qz, tx, ty, tz } 或 null
 */
function extractFixedJointOffset(urdfRobot, jointNames, endLinkName, mdhFrameInfo = null) {
    if (!urdfRobot || !endLinkName) return null;
    
    const joints = urdfRobot.joints || {};
    let fixedJoint = null;
    let fixedJointName = null;
    
    console.log(`\n  查找fixed关节（目标end_link: ${endLinkName}）...`);
    console.log(`  可用关节数: ${Object.keys(joints).length}`);
    
    // 查找指向endLinkName的fixed关节
    for (const name of Object.keys(joints)) {
        const j = joints[name];
        
        // 调试：打印关节信息
        const jointType = j.jointType || j.type || 'unknown';
        
        // 先获取parent链接名称（用于后续判断）
        let parentName = null;
        if (typeof j.parent === 'string') {
            parentName = j.parent;
        } else if (j.parent && typeof j.parent === 'object' && j.parent.name) {
            parentName = j.parent.name;
        } else if (j.parentLink) {
            parentName = j.parentLink;
        } else if (j.parent && typeof j.parent === 'object') {
            parentName = j.parent.link || j.parent.linkName || null;
        }
        
        // 多种方式获取child链接名称
        let childName = null;
        
        // 方法1: 直接字符串属性
        if (typeof j.child === 'string') {
            childName = j.child;
        } else if (j.childLink && typeof j.childLink === 'string') {
            childName = j.childLink;
        }
        // 方法2: 对象属性
        else if (j.child && typeof j.child === 'object') {
            childName = j.child.name || j.child.link || j.child.linkName || null;
            
            // 如果还是null，可能是URDF loader的THREE.Object3D，尝试通过urdfRobot.links反向查找
            if (!childName && urdfRobot.links) {
                const linkNames = Object.keys(urdfRobot.links);
                const matchingLink = linkNames.find(linkName => {
                    const linkObj = urdfRobot.links[linkName];
                    return linkObj === j.child || (linkObj && linkObj.uuid === j.child?.uuid);
                });
                if (matchingLink) {
                    childName = matchingLink;
                }
            }
        }
        
        // 方法3: 如果childName仍然是null，尝试通过urdfRobot.links反向查找
        // URDF loader可能将child存储为THREE.Object3D，需要通过场景图查找
        if (!childName && urdfRobot.links && j.child) {
            const linkNames = Object.keys(urdfRobot.links);
            for (const linkName of linkNames) {
                const linkObj = urdfRobot.links[linkName];
                if (linkObj === j.child || (linkObj && j.child && linkObj.uuid === j.child.uuid)) {
                    childName = linkName;
                    break;
                }
            }
        }
        
        // 方法4: 尝试从joint的children属性查找（THREE.js场景图）
        if (!childName && j.children && Array.isArray(j.children) && j.children.length > 0) {
            // 尝试从children中找到匹配的link
            for (const childObj of j.children) {
                if (childObj && urdfRobot.links) {
                    const linkNames = Object.keys(urdfRobot.links);
                    for (const linkName of linkNames) {
                        if (urdfRobot.links[linkName] === childObj) {
                            childName = linkName;
                            break;
                        }
                    }
                    if (childName) break;
                }
            }
        }
        
        // 检查是否是fixed关节
        const isFixed = jointType === 'fixed' || jointType === 0 || (j.jointType !== undefined && j.jointType === 'fixed');
        
        // 方法5: 如果以上都失败，打印所有属性以便调试
        if (!childName && isFixed) {
            console.log(`      调试：打印joint所有属性键:`, Object.keys(j));
            console.log(`      调试：j.child =`, j.child);
            console.log(`      调试：j.childLink =`, j.childLink);
            console.log(`      调试：j.children =`, j.children);
            
            // 尝试通过parent链接和已知的child链接名称匹配
            // 我们知道ee_fixed_joint的parent是wrist_3_link，child应该是ee_link
            if (parentName === 'wrist_3_link' && name === 'ee_fixed_joint') {
                // 直接使用已知的child链接名称
                childName = 'ee_link';
                console.log(`      使用已知映射: ee_fixed_joint -> ee_link`);
            }
        }
        
        if (isFixed) {
            console.log(`  - 检查fixed关节: ${name}`);
            console.log(`    type: ${jointType}, child: ${childName || 'null'}, parent: ${parentName || 'unknown'}`);
            
            // 检查child是否匹配
            if (childName === endLinkName) {
                fixedJoint = j;
                fixedJointName = name;
                console.log(`    ✓ 匹配！childName=${childName}, endLinkName=${endLinkName}`);
                break;
            } else if (childName) {
                console.log(`    ✗ 不匹配: childName=${childName}, endLinkName=${endLinkName}`);
            }
        }
    }
    
    if (!fixedJoint) {
        console.log(`  ⊘ 未找到指向 ${endLinkName} 的fixed关节`);
        console.log(`  调试：已检查所有 ${Object.keys(joints).length} 个关节`);
        // 打印所有fixed关节以供调试
        const allFixed = Object.keys(joints).filter(name => {
            const j = joints[name];
            const jointType = j.jointType || j.type;
            return jointType === 'fixed' || jointType === 0;
        });
        if (allFixed.length > 0) {
            console.log(`  发现的fixed关节: ${allFixed.join(', ')}`);
            allFixed.forEach(name => {
                const j = joints[name];
                // 安全地获取child名称
                let childStr = 'null';
                if (typeof j.child === 'string') {
                    childStr = j.child;
                } else if (j.child && typeof j.child === 'object' && j.child.name) {
                    childStr = j.child.name;
                } else if (j.childLink) {
                    childStr = j.childLink;
                }
                console.log(`    - ${name}: child=${childStr}, childType=${typeof j.child}, childLink=${j.childLink || 'undefined'}`);
                
                // 如果是URDF loader的对象，可能child是一个THREE.Object3D，需要通过urdfRobot.links查找
                if (j.child && typeof j.child === 'object' && urdfRobot.links) {
                    // 尝试通过链接名称匹配
                    const linkNames = Object.keys(urdfRobot.links);
                    const matchingLink = linkNames.find(linkName => urdfRobot.links[linkName] === j.child);
                    if (matchingLink) {
                        console.log(`      → 通过links找到匹配: ${matchingLink}`);
                    }
                }
            });
        }
        return null;
    }
    
    // 获取parent链接名称
    let parentName = 'unknown';
    if (typeof fixedJoint.parent === 'string') {
        parentName = fixedJoint.parent;
    } else if (fixedJoint.parent && typeof fixedJoint.parent === 'object' && fixedJoint.parent.name) {
        parentName = fixedJoint.parent.name;
    } else if (fixedJoint.parentLink) {
        parentName = fixedJoint.parentLink;
    } else if (fixedJoint.parent && typeof fixedJoint.parent === 'object') {
        parentName = fixedJoint.parent.link || fixedJoint.parent.linkName || 'unknown';
    }
    
    console.log(`  ✓ 找到fixed关节: ${fixedJointName} (从 ${parentName} 到 ${endLinkName})`);
    
    return convertJointPoseToDQ(fixedJoint, urdfRobot, parentName, mdhFrameInfo);
}

/**
 * 从父链接查找fixed joint
 * @param {Object} urdfRobot - URDF机器人对象
 * @param {string} parentLinkName - 父链接名称
 * @param {Object} mdhFrameInfo - MDH最后一个关节的坐标系信息
 * @returns {Object|null} { qw, qx, qy, qz, tx, ty, tz } 或 null
 */
function findFixedJointFromParentLink(urdfRobot, parentLinkName, mdhFrameInfo = null, endLinkName = null) {
    if (!urdfRobot || !parentLinkName) return null;
    
    const joints = urdfRobot.joints || {};
    let fixedJoint = null;
    let fixedJointName = null;
    
    console.log(`\n  查找fixed关节（从parent_link: ${parentLinkName}${endLinkName ? `, 到end_link: ${endLinkName}` : ''}）...`);
    
    // 查找parent为parentLinkName的fixed关节
    for (const name of Object.keys(joints)) {
        const j = joints[name];
        const jointType = j.jointType || j.type || 'unknown';
        const isFixed = jointType === 'fixed' || jointType === 0 || (j.jointType !== undefined && j.jointType === 'fixed');
        
        if (!isFixed) continue;
        
        // 获取parent链接名称
        let parentName = null;
        if (typeof j.parent === 'string') {
            parentName = j.parent;
        } else if (j.parent && typeof j.parent === 'object' && j.parent.name) {
            parentName = j.parent.name;
        } else if (j.parentLink) {
            parentName = j.parentLink;
        } else if (j.parent && typeof j.parent === 'object') {
            parentName = j.parent.link || j.parent.linkName || null;
        }
        
        // 获取child链接名称（用于验证是否指向endLinkName）
        // 使用与computeEffectorOffsetFromForwardKinematics相同的健壮逻辑
        let childName = null;
        
        // 方法1: 直接字符串属性
        if (typeof j.child === 'string') {
            childName = j.child;
        } else if (j.childLink && typeof j.childLink === 'string') {
            childName = j.childLink;
        }
        // 方法2: 对象属性
        else if (j.child && typeof j.child === 'object') {
            childName = j.child.name || j.child.link || j.child.linkName || null;
            
            // 如果还是null，可能是URDF loader的THREE.Object3D，尝试通过urdfRobot.links反向查找
            if (!childName && urdfRobot.links) {
                const linkNames = Object.keys(urdfRobot.links);
                const matchingLink = linkNames.find(linkName => {
                    const linkObj = urdfRobot.links[linkName];
                    return linkObj === j.child || (linkObj && linkObj.uuid === j.child?.uuid);
                });
                if (matchingLink) {
                    childName = matchingLink;
                }
            }
        }
        
        // 方法3: 如果childName仍然是null，尝试通过urdfRobot.links反向查找
        if (!childName && urdfRobot.links && j.child) {
            const linkNames = Object.keys(urdfRobot.links);
            for (const linkName of linkNames) {
                const linkObj = urdfRobot.links[linkName];
                if (linkObj === j.child || (linkObj && j.child && linkObj.uuid === j.child.uuid)) {
                    childName = linkName;
                    break;
                }
            }
        }
        
        // 方法4: 尝试从joint的children属性查找（THREE.js场景图）
        if (!childName && j.children && Array.isArray(j.children) && j.children.length > 0) {
            for (const childObj of j.children) {
                if (childObj && urdfRobot.links) {
                    const linkNames = Object.keys(urdfRobot.links);
                    for (const linkName of linkNames) {
                        if (urdfRobot.links[linkName] === childObj) {
                            childName = linkName;
                            break;
                        }
                    }
                    if (childName) break;
                }
            }
        }
        
        // 方法5: 如果以上都失败，尝试通过已知的映射
        // 我们知道ee_fixed_joint的parent是wrist_3_link，child应该是ee_link
        if (!childName && parentName === 'wrist_3_link' && name.includes('ee_fixed_joint')) {
            childName = 'ee_link';
            console.log(`    使用已知映射: ${name} -> ee_link`);
        }
        
        console.log(`  - 检查fixed关节: ${name}, parent=${parentName}, child=${childName}`);
        
        // 检查parent是否匹配，并且如果指定了endLinkName，child也要匹配
        const parentMatches = parentName === parentLinkName ||
            (typeof parentName === 'string' && parentName.includes(parentLinkName));
        
        const childMatches = !endLinkName || childName === endLinkName ||
            (typeof childName === 'string' && childName.includes(endLinkName));
        
        if (parentMatches && childMatches) {
            fixedJoint = j;
            fixedJointName = name;
            console.log(`    ✓ 匹配！`);
            break; // 找到第一个就返回
        }
    }
    
    if (!fixedJoint) {
        console.log(`  ⊘ 未找到从 ${parentLinkName}${endLinkName ? ` 到 ${endLinkName}` : ''} 的fixed关节`);
        return null;
    }
    
    // 再次获取child link名称（用于日志）
    let childLink = null;
    if (typeof fixedJoint.child === 'string') {
        childLink = fixedJoint.child;
    } else if (fixedJoint.childLink && typeof fixedJoint.childLink === 'string') {
        childLink = fixedJoint.childLink;
    } else if (fixedJoint.child && typeof fixedJoint.child === 'object') {
        childLink = fixedJoint.child.name || fixedJoint.child.link || fixedJoint.child.linkName || null;
        
        // 如果还是null，尝试反向查找
        if (!childLink && urdfRobot.links) {
            const linkNames = Object.keys(urdfRobot.links);
            const matchingLink = linkNames.find(linkName => {
                const linkObj = urdfRobot.links[linkName];
                return linkObj === fixedJoint.child || (linkObj && linkObj.uuid === fixedJoint.child?.uuid);
            });
            if (matchingLink) {
                childLink = matchingLink;
            }
        }
    }
    
    // 如果还是null，尝试通过children查找
    if (!childLink && fixedJoint.children && Array.isArray(fixedJoint.children)) {
        for (const childObj of fixedJoint.children) {
            if (childObj && urdfRobot.links) {
                const linkNames = Object.keys(urdfRobot.links);
                for (const linkName of linkNames) {
                    if (urdfRobot.links[linkName] === childObj) {
                        childLink = linkName;
                        break;
                    }
                }
                if (childLink) break;
            }
        }
    }
    
    console.log(`  ✓ 找到fixed关节: ${fixedJointName} (从 ${parentLinkName} 到 ${childLink || 'unknown'})`);
    
    // 检查mdhFrameInfo是否有效
    if (!mdhFrameInfo || !mdhFrameInfo.lastX || !mdhFrameInfo.lastY || !mdhFrameInfo.lastZ) {
        console.warn(`  ⚠ mdhFrameInfo无效，无法转换fixed joint偏移`);
        return null;
    }
    
    return convertJointPoseToDQ(fixedJoint, urdfRobot, parentLinkName, mdhFrameInfo);
}

/**
 * 将关节位姿转换为DQ格式（Z-up坐标系）
 * 
 * 关键问题：fixed joint的偏移是相对于其parent link（URDF坐标系）的，
 * 而setEffectorPose期望的是相对于最后一个关节的MDH坐标系的偏移。
 * 这两个坐标系可能不一致，需要进行坐标系转换。
 * 
 * @param {Object} joint - URDF关节对象
 * @param {Object} urdfRobot - URDF机器人对象（用于获取parent link的变换）
 * @param {string} parentLinkName - parent link的名称
 * @param {Object} mdhFrameInfo - MDH最后一个关节的坐标系信息 {lastOrigin, lastX, lastY, lastZ}
 * @returns {Object} { qw, qx, qy, qz, tx, ty, tz }
 */
function convertJointPoseToDQ(joint, urdfRobot = null, parentLinkName = null, mdhFrameInfo = null) {
    // 获取URDF原始位姿（Y-up坐标系，相对于parent link）
    const posY = joint.position ? joint.position.clone() : new THREE.Vector3();
    const quatY = joint.quaternion ? joint.quaternion.clone() : new THREE.Quaternion();
    
    console.log(`    URDF Y-up 位移 (相对于parent link ${parentLinkName}): (${posY.x.toFixed(4)}, ${posY.y.toFixed(4)}, ${posY.z.toFixed(4)})`);
    console.log(`    URDF Y-up 四元数: (${quatY.w.toFixed(4)}, ${quatY.x.toFixed(4)}, ${quatY.y.toFixed(4)}, ${quatY.z.toFixed(4)})`);
    
    // 第一步：从Y-up转换到Z-up（整体坐标系转换）
    const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
    const posZUp = posY.clone().applyQuaternion(qx);
    const quatZUp = qx.clone().multiply(quatY);
    
    console.log(`    步骤1: Y-up -> Z-up 位移: (${posZUp.x.toFixed(4)}, ${posZUp.y.toFixed(4)}, ${posZUp.z.toFixed(4)})`);
    
    // 第二步：如果提供了MDH坐标系信息，需要从parent link坐标系转换到MDH最后一个关节坐标系
    if (mdhFrameInfo && urdfRobot && parentLinkName) {
        console.log(`    步骤2: 将偏移从parent link坐标系转换到MDH最后一个关节坐标系...`);
        
        // 获取parent link在世界坐标系（Y-up）中的变换
        const parentLink = urdfRobot.links[parentLinkName];
        if (parentLink) {
            parentLink.updateMatrixWorld(true);
            
            // parent link的变换矩阵（Y-up）
            const parentMatrixY = parentLink.matrixWorld.clone();
            
            // 转换为Z-up
            const qxTransform = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
            const parentMatrixZ = qxTransform.clone().multiply(parentMatrixY);
            
            // 构建parent link坐标系（Z-up）的变换矩阵
            const parentOriginZ = new THREE.Vector3().setFromMatrixPosition(parentMatrixZ);
            const parentRotZ = new THREE.Matrix4().extractRotation(parentMatrixZ);
            
            // parent link的X、Y、Z轴（Z-up）
            const parentX = new THREE.Vector3(1, 0, 0).transformDirection(parentRotZ);
            const parentY = new THREE.Vector3(0, 1, 0).transformDirection(parentRotZ);
            const parentZ = new THREE.Vector3(0, 0, 1).transformDirection(parentRotZ);
            
            console.log(`      Parent link (${parentLinkName}) Z-up坐标系:`);
            console.log(`        原点: (${parentOriginZ.x.toFixed(4)}, ${parentOriginZ.y.toFixed(4)}, ${parentOriginZ.z.toFixed(4)})`);
            console.log(`        X轴: (${parentX.x.toFixed(4)}, ${parentX.y.toFixed(4)}, ${parentX.z.toFixed(4)})`);
            console.log(`        Y轴: (${parentY.x.toFixed(4)}, ${parentY.y.toFixed(4)}, ${parentY.z.toFixed(4)})`);
            console.log(`        Z轴: (${parentZ.x.toFixed(4)}, ${parentZ.y.toFixed(4)}, ${parentZ.z.toFixed(4)})`);
            
            // 将fixed joint的偏移（在parent link坐标系中）转换为MDH坐标系
            // 关键：需要计算parent link坐标系到MDH最后一个关节坐标系的变换
            
            // 1. 构建parent link坐标系到世界的变换矩阵（Z-up）
            // parent link的旋转矩阵（Z-up）
            const parentRotQuat = new THREE.Quaternion().setFromRotationMatrix(parentRotZ);
            
            // 2. 构建MDH最后一个关节坐标系到世界的变换矩阵（Z-up）
            // 检查mdhFrameInfo是否有效
            if (!mdhFrameInfo || !mdhFrameInfo.lastX || !mdhFrameInfo.lastY || !mdhFrameInfo.lastZ) {
                console.warn(`    ⚠ mdhFrameInfo无效，无法转换fixed joint偏移`);
                return null;
            }
            
            const mdhRotation = new THREE.Matrix4();
            mdhRotation.makeBasis(mdhFrameInfo.lastX, mdhFrameInfo.lastY, mdhFrameInfo.lastZ);
            const mdhRotQuat = new THREE.Quaternion().setFromRotationMatrix(mdhRotation);
            
            // 3. 计算parent link坐标系到MDH坐标系的相对变换
            // parent_link_frame = T_parent_to_world
            // mdh_frame = T_mdh_to_world
            // offset_in_mdh = T_world_to_mdh * T_parent_to_world * offset_in_parent
            //                = T_mdh_to_world^-1 * T_parent_to_world * offset_in_parent
            
            // 构建变换矩阵
            const parentToWorld = new THREE.Matrix4();
            parentToWorld.compose(parentOriginZ, parentRotQuat, new THREE.Vector3(1, 1, 1));
            
            const mdhToWorld = new THREE.Matrix4();
            mdhToWorld.compose(mdhFrameInfo.lastOrigin, mdhRotQuat, new THREE.Vector3(1, 1, 1));
            
            // 计算相对变换：parent -> world -> MDH
            const mdhToWorldInverse = mdhToWorld.clone().invert();
            const parentToMDH = mdhToWorldInverse.clone().multiply(parentToWorld);
            
            // 4. 将偏移从parent link坐标系转换到MDH坐标系
            const offsetInParentFrame = posZUp.clone();
            const offsetInMDH = offsetInParentFrame.clone().applyMatrix4(parentToMDH);
            
            // 5. 四元数转换：offset_quat_in_mdh = mdh_rot^-1 * parent_rot * offset_quat_in_parent
            const offsetQuatInParentFrame = quatZUp.clone();
            const offsetQuatInMDH = mdhRotQuat.clone().invert().multiply(parentRotQuat).multiply(offsetQuatInParentFrame);
            
            console.log(`      MDH最后一个关节坐标系:`);
            console.log(`        原点: (${mdhFrameInfo.lastOrigin.x.toFixed(4)}, ${mdhFrameInfo.lastOrigin.y.toFixed(4)}, ${mdhFrameInfo.lastOrigin.z.toFixed(4)})`);
            console.log(`        X轴: (${mdhFrameInfo.lastX.x.toFixed(4)}, ${mdhFrameInfo.lastX.y.toFixed(4)}, ${mdhFrameInfo.lastX.z.toFixed(4)})`);
            console.log(`        Y轴: (${mdhFrameInfo.lastY.x.toFixed(4)}, ${mdhFrameInfo.lastY.y.toFixed(4)}, ${mdhFrameInfo.lastY.z.toFixed(4)})`);
            console.log(`        Z轴: (${mdhFrameInfo.lastZ.x.toFixed(4)}, ${mdhFrameInfo.lastZ.y.toFixed(4)}, ${mdhFrameInfo.lastZ.z.toFixed(4)})`);
            
            // 检查两个坐标系的原点距离
            const originDiff = parentOriginZ.clone().sub(mdhFrameInfo.lastOrigin);
            const originDistance = originDiff.length();
            console.log(`      坐标系原点距离: ${originDistance.toFixed(4)}`);
            console.log(`      原点差值: (${originDiff.x.toFixed(4)}, ${originDiff.y.toFixed(4)}, ${originDiff.z.toFixed(4)})`);
            
            // 检查坐标系方向差异
            const parentRotMat = new THREE.Matrix3().setFromMatrix4(parentRotZ);
            const mdhRotMat = new THREE.Matrix3().setFromMatrix4(mdhRotation);
            const rotDiff = mdhRotMat.clone().multiply(parentRotMat.clone().transpose());
            const rotDiffQuat = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().setFromMatrix3(rotDiff));
            const rotDiffAngle = 2 * Math.acos(Math.abs(rotDiffQuat.w));
            console.log(`      坐标系旋转差异: ${(rotDiffAngle * 180 / Math.PI).toFixed(4)}°`);
            
            console.log(`      转换后的偏移 (相对于MDH坐标系): (${offsetInMDH.x.toFixed(4)}, ${offsetInMDH.y.toFixed(4)}, ${offsetInMDH.z.toFixed(4)})`);
            console.log(`      转换后的四元数: (${offsetQuatInMDH.w.toFixed(4)}, ${offsetQuatInMDH.x.toFixed(4)}, ${offsetQuatInMDH.y.toFixed(4)}, ${offsetQuatInMDH.z.toFixed(4)})`);
            
            return {
                qw: offsetQuatInMDH.w,
                qx: offsetQuatInMDH.x,
                qy: offsetQuatInMDH.y,
                qz: offsetQuatInMDH.z,
                tx: offsetInMDH.x,
                ty: offsetInMDH.y,
                tz: offsetInMDH.z
            };
        } else {
            console.warn(`    ⚠ 未找到parent link ${parentLinkName}，使用简单转换`);
        }
    }
    
    // 如果没有MDH坐标系信息，只进行Y-up到Z-up的转换
    console.log(`    最终偏移 (Z-up，假设坐标系一致): (${posZUp.x.toFixed(4)}, ${posZUp.y.toFixed(4)}, ${posZUp.z.toFixed(4)})`);
    console.log(`    最终四元数: (${quatZUp.w.toFixed(4)}, ${quatZUp.x.toFixed(4)}, ${quatZUp.y.toFixed(4)}, ${quatZUp.z.toFixed(4)})`);
    
    return {
        qw: quatZUp.w,
        qx: quatZUp.x,
        qy: quatZUp.y,
        qz: quatZUp.z,
        tx: posZUp.x,
        ty: posZUp.y,
        tz: posZUp.z
    };
}

/**
 * 从关节链数据计算 Modified DH 参数
 * 
 * 基于正确的几何算法：
 * - theta: 绕z_i轴，从x_{i-1}转到x_i的角度
 * - d: 沿z_i轴，从原点o_{i-1}到o_i的距离
 * - a: 沿x_{i-1}轴，从原点o_{i-1}到o_i的距离
 * - alpha: 绕x_{i-1}轴，从z_{i-1}转到z_i的角度
 * 
 * 参考: URDFly mdh_dialog.py
 * 
 * 注意：MDH参数只计算旋转关节链，不包含fixed joint偏移
 * fixed joint的偏移应该通过setEffectorPose单独设置
 * 
 * @param {Array} jointData - 关节数据数组（只包含旋转关节）
 * @param {THREE.Vector3} endEffectorPosition - 已废弃，不再使用（保留以兼容）
 * @returns {Object} { mdhParams: [[θ, d, a, α], ...], mdhFrameInfo: {lastOrigin, lastX, lastY, lastZ, lastZAxis} }
 */
function computeMDHFromJointChain(jointData, endEffectorPosition = null, baseConfig = null) {
    const numJoints = jointData.length;
    
    // 步骤1: 获取每个关节在世界坐标系中的位置和轴向
    const jointPositions = [];
    const jointZAxes = [];  // 关节的Z轴（旋转轴）
    const jointXAxes = [];  // 关节的X轴（初始方向）
    
    let cumulativeMatrix = new THREE.Matrix4();  // 累积变换矩阵
    
    // 基座配置（手动设置或使用默认值）
    let baseOrigin, baseZAxis, baseXAxis;
    if (baseConfig && baseConfig.origin && baseConfig.z_axis && baseConfig.x_axis) {
        // 使用手动配置的基座
        baseOrigin = new THREE.Vector3(
            baseConfig.origin[0] || 0,
            baseConfig.origin[1] || 0,
            baseConfig.origin[2] || 0
        );
        baseZAxis = new THREE.Vector3(
            baseConfig.z_axis[0] || 0,
            baseConfig.z_axis[1] || 0,
            baseConfig.z_axis[2] || 1
        ).normalize();
        baseXAxis = new THREE.Vector3(
            baseConfig.x_axis[0] || 1,
            baseConfig.x_axis[1] || 0,
            baseConfig.x_axis[2] || 0
        ).normalize();
        console.log(`  使用手动配置的基座:`);
        console.log(`    原点: (${baseOrigin.x.toFixed(4)}, ${baseOrigin.y.toFixed(4)}, ${baseOrigin.z.toFixed(4)})`);
        console.log(`    Z轴: (${baseZAxis.x.toFixed(4)}, ${baseZAxis.y.toFixed(4)}, ${baseZAxis.z.toFixed(4)})`);
        console.log(`    X轴: (${baseXAxis.x.toFixed(4)}, ${baseXAxis.y.toFixed(4)}, ${baseXAxis.z.toFixed(4)})`);
    } else {
        // 使用默认基座
        baseOrigin = new THREE.Vector3(0, 0, 0);
        baseZAxis = new THREE.Vector3(0, 0, 1);
        baseXAxis = new THREE.Vector3(1, 0, 0);
    }
    
    jointPositions.push(baseOrigin.clone());
    jointZAxes.push(baseZAxis.clone());
    jointXAxes.push(baseXAxis.clone());
    
    for (let i = 0; i < numJoints; i++) {
        const joint = jointData[i];
        
        // 累积变换
        cumulativeMatrix = cumulativeMatrix.clone().multiply(joint.matrix);
        
        // 从累积矩阵提取位置
        const position = new THREE.Vector3();
        position.setFromMatrixPosition(cumulativeMatrix);
        jointPositions.push(position);
        
        // 获取关节轴在世界坐标系中的方向
        // 关节轴axis是在局部坐标系中定义的，需要转换到世界坐标系
        const rotationMatrix = new THREE.Matrix4().extractRotation(cumulativeMatrix);
        const worldAxis = joint.axis.clone().transformDirection(rotationMatrix).normalize();
        jointZAxes.push(worldAxis);
        
        // 初始X轴方向（参考Python line 536-543）
        // 根据关节轴的类型选择不同的X轴方向
        let xAxis;
        const absAxis = new THREE.Vector3(
            Math.abs(joint.axis.x),
            Math.abs(joint.axis.y),
            Math.abs(joint.axis.z)
        );
        
        // 判断关节轴类型（在局部坐标系中）
        if (Math.abs(absAxis.x - 0) < 1e-6 && Math.abs(absAxis.y - 0) < 1e-6 && Math.abs(absAxis.z - 1) < 1e-6) {
            // z轴旋转，使用[1, 0, 0]变换到世界坐标系
            xAxis = new THREE.Vector3(1, 0, 0).transformDirection(rotationMatrix).normalize();
        } else if (Math.abs(absAxis.x - 0) < 1e-6 && Math.abs(absAxis.y - 1) < 1e-6 && Math.abs(absAxis.z - 0) < 1e-6) {
            // y轴旋转，使用[1, 0, 0]变换到世界坐标系
            xAxis = new THREE.Vector3(1, 0, 0).transformDirection(rotationMatrix).normalize();
        } else if (Math.abs(absAxis.x - 1) < 1e-6 && Math.abs(absAxis.y - 0) < 1e-6 && Math.abs(absAxis.z - 0) < 1e-6) {
            // x轴旋转，使用[0, 1, 0]变换到世界坐标系
            xAxis = new THREE.Vector3(0, 1, 0).transformDirection(rotationMatrix).normalize();
        } else {
            // 其他情况，使用任意垂直于Z的向量
        const tempX = new THREE.Vector3(1, 0, 0);
        if (Math.abs(worldAxis.dot(tempX)) > 0.99) {
            tempX.set(0, 1, 0);
        }
            xAxis = new THREE.Vector3().crossVectors(
            worldAxis,
            new THREE.Vector3().crossVectors(tempX, worldAxis)
        ).normalize();
        }
        jointXAxes.push(xAxis);
    }
    
    // 步骤2: 计算MDH坐标系的原点和X轴（参考Python urdf.py的calculate_mdh_origin_position）
    const mdhOrigins = [];  // MDH原点数组
    const mdhZAxes = [];    // MDH Z轴数组（对应Python的mdh_zs）
    const mdhXAxes = [];     // MDH X轴数组（对应Python的mdh_xs）
    
    // 从i=0开始计算MDH坐标系（MDH1, MDH2, ...）
    // 注意：Python中不计算MDH0，直接从MDH1开始
    for (let i = 0; i < numJoints; i++) {
        const z_i = jointZAxes[i];         // i=0时是baseZ
        const z_next = jointZAxes[i + 1];  // i=0时是joint1Z
        const p_i = jointPositions[i];     // i=0时是base位置
        const p_next = jointPositions[i + 1];  // i=0时是joint1位置
        
        // 调试关节4的MDH原点计算
        if (i === 4) {
            console.log(`\n计算MDH原点${i} (基于关节${i}和关节${i+1}):`);
            console.log(`  z_i(${i}): (${z_i.x.toFixed(4)}, ${z_i.y.toFixed(4)}, ${z_i.z.toFixed(4)})`);
            console.log(`  z_next(${i+1}): (${z_next.x.toFixed(4)}, ${z_next.y.toFixed(4)}, ${z_next.z.toFixed(4)})`);
            console.log(`  p_i(${i}): (${p_i.x.toFixed(4)}, ${p_i.y.toFixed(4)}, ${p_i.z.toFixed(4)})`);
            console.log(`  p_next(${i+1}): (${p_next.x.toFixed(4)}, ${p_next.y.toFixed(4)}, ${p_next.z.toFixed(4)})`);
        }
        
        // 计算MDH原点位置（完全参照Python的calculate_mdh_origin_position）
        // zi: z_i, zi_next: z_next, pi: p_i, pi_next: p_next
        let mdhOrigin, mdhX, caseType, commonPerpendicular;
        
        // 检查zi和zi_next是否平行或重合
        const crossZ = new THREE.Vector3().crossVectors(z_i, z_next);
        const crossZLen = crossZ.length();
        
        if (crossZLen < 1e-6) {
            // 情况1/3: Z轴平行或重合
            const diff = p_next.clone().sub(p_i);
            const crossDiff = new THREE.Vector3().crossVectors(diff, z_i);
            
            if (crossDiff.length() < 1e-6) {
                // 情况1: 重合在同一直线上
                mdhOrigin = p_i.clone();
                caseType = 'coincident';
                commonPerpendicular = null;
                // X轴使用当前关节的初始X轴（在计算MDH参数时会根据case设置）
            } else {
                // 情况3: 平行但不重合
                mdhOrigin = p_i.clone();
                caseType = 'parallel';
                // 计算公垂线的两个端点
                const proj = diff.dot(z_i);
                const point1 = p_i.clone();
                const point2 = p_next.clone().sub(z_i.clone().multiplyScalar(proj));
                commonPerpendicular = [point1, point2];
            }
        } else {
            // 检查是否相交
            const diff = p_next.clone().sub(p_i);
            const distance = Math.abs(diff.dot(crossZ)) / crossZLen;
            
            if (distance < 1e-6) {
                // 情况2: 相交
                // 解方程组找到交点: p_i + t*zi = p_next + s*zi_next
                // A = [zi, -zi_next], b = p_next - p_i
                // 使用最小二乘法求解（参考Python line 602-604）
                const ziArray = [z_i.x, z_i.y, z_i.z];
                const ziNextArray = [-z_next.x, -z_next.y, -z_next.z];
                const bArray = [diff.x, diff.y, diff.z];
                
                // 构建矩阵A和向量b
                // 使用最小二乘法：A^T * A * [t, s]^T = A^T * b
                const ATA = [
                    [z_i.dot(z_i), z_i.dot(z_next)],
                    [z_i.dot(z_next), z_next.dot(z_next)]
                ];
                const ATb = [
                    diff.dot(z_i),
                    -diff.dot(z_next)
                ];
                
                // 求解 [t, s]
                const det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0];
                let t = 0, s = 0;
                if (Math.abs(det) > 1e-9) {
                    t = (ATA[1][1] * ATb[0] - ATA[0][1] * ATb[1]) / det;
                    s = (ATA[0][0] * ATb[1] - ATA[1][0] * ATb[0]) / det;
                }
                
                mdhOrigin = p_i.clone().add(z_i.clone().multiplyScalar(t));
                caseType = 'intersect';
                commonPerpendicular = [mdhOrigin.clone(), mdhOrigin.clone()];
            } else {
                // 情况4: 不相交也不平行 (异面直线)
                const n = crossZ.clone().normalize();
                
                // 建立方程组: p_i + t*zi = p_next + s*zi_next + u*n
                // 即: t*zi - s*zi_next - u*n = p_next - p_i
                // 参考Python line 633-635：使用最小二乘法求解 [t, s, u]
                // 构建矩阵A = [zi, -zi_next, n], b = diff
                const A = [
                    [z_i.x, -z_next.x, n.x],
                    [z_i.y, -z_next.y, n.y],
                    [z_i.z, -z_next.z, n.z]
                ];
                
                // 使用最小二乘法：A^T * A * x = A^T * b
                // 计算A^T * A
                const ATA = [
                    [z_i.dot(z_i), -z_i.dot(z_next), z_i.dot(n)],
                    [-z_i.dot(z_next), z_next.dot(z_next), -z_next.dot(n)],
                    [z_i.dot(n), -z_next.dot(n), n.dot(n)]
                ];
                
                // 计算A^T * b
                const ATb = [
                    diff.dot(z_i),
                    -diff.dot(z_next),
                    diff.dot(n)
                ];
                
                // 求解3x3线性方程组：ATA * [t, s, u]^T = ATb
                // 使用Cramer法则或高斯消元法
                const det = ATA[0][0] * (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) -
                           ATA[0][1] * (ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) +
                           ATA[0][2] * (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]);
                
                let t = 0, s = 0;
                if (Math.abs(det) > 1e-9) {
                    // 计算t（替换第一列）
                    const det_t = ATb[0] * (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) -
                                 ATA[0][1] * (ATb[1] * ATA[2][2] - ATA[1][2] * ATb[2]) +
                                 ATA[0][2] * (ATb[1] * ATA[2][1] - ATA[1][1] * ATb[2]);
                    t = det_t / det;
                    
                    // 计算s（替换第二列）
                    const det_s = ATA[0][0] * (ATb[1] * ATA[2][2] - ATA[1][2] * ATb[2]) -
                                 ATb[0] * (ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) +
                                 ATA[0][2] * (ATA[1][0] * ATb[2] - ATb[1] * ATA[2][0]);
                    s = det_s / det;
                }
                
                mdhOrigin = p_i.clone().add(z_i.clone().multiplyScalar(t));
                caseType = 'skew';
                const point1 = mdhOrigin.clone();
                const point2 = p_next.clone().add(z_next.clone().multiplyScalar(s));
                commonPerpendicular = [point1, point2];
            }
        }
        
        // 根据case类型选择X轴（参考Python line 730-742）
        if (caseType === 'coincident') {
            // 使用当前关节的初始X轴
            mdhX = jointXAxes[i].clone();
        } else if (caseType === 'skew' || caseType === 'intersect') {
            // X轴是zi和zi_next的叉积方向
            mdhX = crossZ.clone().normalize();
        } else if (caseType === 'parallel') {
            // X轴是公垂线方向
            mdhX = commonPerpendicular[1].clone().sub(commonPerpendicular[0]).normalize();
        }
        
        // 调试关节4的MDH原点计算结果
        if (i === 4) {
            console.log(`  计算得到的mdhOrigin: (${mdhOrigin.x.toFixed(4)}, ${mdhOrigin.y.toFixed(4)}, ${mdhOrigin.z.toFixed(4)})`);
            console.log(`  计算得到的mdhX: (${mdhX.x.toFixed(4)}, ${mdhX.y.toFixed(4)}, ${mdhX.z.toFixed(4)})`);
            // 验证mdhOrigin是否在z_i上
            const vecFromPi = mdhOrigin.clone().sub(p_i);
            const projection = vecFromPi.dot(z_i);
            const perpendicular = vecFromPi.clone().sub(z_i.clone().multiplyScalar(projection));
            console.log(`  验证: mdhOrigin到p_i的向量在z_i上的投影长度 = ${projection.toFixed(4)}`);
            console.log(`  验证: 垂直分量长度 = ${perpendicular.length().toFixed(6)} (应接近0)`);
        }
        
        mdhOrigins.push(mdhOrigin);
        mdhZAxes.push(z_i.clone());  // 存储当前关节的Z轴（对应Python的mdh_zs.append(joint_vector)）
        mdhXAxes.push(mdhX);
    }
    
    // 添加最后一个关节的坐标系（MDH只计算到最后一个旋转关节）
    // 注意：不要使用endEffectorPosition，fixed joint的偏移应该通过setEffectorPose单独设置
    // MDH参数应该只描述旋转关节链的几何关系
    // 参考Python line 744-747：直接使用最后一个关节的位置、Z轴和X轴
    mdhOrigins.push(jointPositions[numJoints]);
    mdhZAxes.push(jointZAxes[numJoints]);  // 存储最后一个关节的Z轴
    mdhXAxes.push(jointXAxes[numJoints]);
    
    // 调试输出
    console.log('\n=== 调试信息 ===');
    console.log(`mdhOrigins数量: ${mdhOrigins.length}, jointZAxes数量: ${jointZAxes.length}, mdhXAxes数量: ${mdhXAxes.length}`);
    console.log('所有MDH原点:');
    for (let i = 0; i < mdhOrigins.length; i++) {
        console.log(`  MDH原点${i}: (${mdhOrigins[i].x.toFixed(4)}, ${mdhOrigins[i].y.toFixed(4)}, ${mdhOrigins[i].z.toFixed(4)})`);
    }
    console.log('所有关节位置:');
    for (let i = 0; i < jointPositions.length; i++) {
        console.log(`  Joint位置${i}: (${jointPositions[i].x.toFixed(4)}, ${jointPositions[i].y.toFixed(4)}, ${jointPositions[i].z.toFixed(4)})`);
    }
    
    // 步骤3: 计算MDH参数
    const mdhParams = [];
    
    for (let i = 0; i < numJoints; i++) {
        const o_prev = mdhOrigins[i];          // o_{i-1} (MDH原点)
        const o_curr = mdhOrigins[i + 1];      // o_i (MDH原点)
        const z_prev = mdhZAxes[i];            // z_{i-1} (使用mdhZAxes，对应Python的mdh_zs[i])
        const z_curr = mdhZAxes[i + 1];        // z_i (使用mdhZAxes，对应Python的mdh_zs[i+1])
        const x_prev = mdhXAxes[i];            // x_{i-1}
        const x_curr = mdhXAxes[i + 1];        // x_i
        
        let theta = 0;
        let d = 0;
        let a = 0;
        let alpha = 0;
        
        // 计算 theta: 绕zi轴，从x_{i-1}到x_i的角度（参考Python line 778-792）
        // Project xi-1 and xi onto the plane perpendicular to zi
        const p_prev = x_prev.clone().sub(
            z_curr.clone().multiplyScalar(x_prev.dot(z_curr))
        );
        const pi = x_curr.clone().sub(
            z_curr.clone().multiplyScalar(x_curr.dot(z_curr))
        );
        
        // Normalize the projected vectors (注意：如果投影长度为0，归一化会返回零向量)
        const len_prev = p_prev.length();
        const len_curr = pi.length();
        
        if (len_prev < 1e-9 || len_curr < 1e-9) {
            // 投影长度接近0，说明x_prev或x_curr与z_curr平行，theta为0
            theta = 0;
        } else {
            // Normalize (参考Python line 784-785)
            const pi_norm = pi.clone().normalize();
            const p_prev_norm = p_prev.clone().normalize();
            
            // Calculate cosine and sine of the angle (参考Python line 788-789)
            const cos_theta = p_prev_norm.dot(pi_norm);
            const sin_theta = new THREE.Vector3().crossVectors(p_prev_norm, pi_norm).dot(z_curr);
            
            // Compute the directed angle using atan2 (参考Python line 792)
            theta = Math.atan2(sin_theta, cos_theta);
        }
        
        // 计算位移向量（从前一个关节到当前关节）
        const vec = o_curr.clone().sub(o_prev);
        
        // Modified DH (MDH) 约定（参考Python line 795, 798）：
        // d_i = 沿zi轴，从o_{i-1}到o_i的距离
        // a_i = 沿x_{i-1}轴，从z_{i-1}到z_i的距离
        d = vec.dot(z_curr);
        a = vec.dot(x_prev);
        
        // 调试所有关节的MDH参数计算
        console.log(`\n关节${i+1}的MDH参数计算:`);
        console.log(`  o_prev(${i}): (${o_prev.x.toFixed(4)}, ${o_prev.y.toFixed(4)}, ${o_prev.z.toFixed(4)})`);
        console.log(`  o_curr(${i+1}): (${o_curr.x.toFixed(4)}, ${o_curr.y.toFixed(4)}, ${o_curr.z.toFixed(4)})`);
        console.log(`  vec: (${vec.x.toFixed(4)}, ${vec.y.toFixed(4)}, ${vec.z.toFixed(4)})`);
        console.log(`  z_prev(${i}): (${z_prev.x.toFixed(4)}, ${z_prev.y.toFixed(4)}, ${z_prev.z.toFixed(4)})`);
        console.log(`  z_curr(${i+1}): (${z_curr.x.toFixed(4)}, ${z_curr.y.toFixed(4)}, ${z_curr.z.toFixed(4)})`);
        console.log(`  x_prev(${i}): (${x_prev.x.toFixed(4)}, ${x_prev.y.toFixed(4)}, ${x_prev.z.toFixed(4)})`);
        console.log(`  x_curr(${i+1}): (${x_curr.x.toFixed(4)}, ${x_curr.y.toFixed(4)}, ${x_curr.z.toFixed(4)})`);
        console.log(`  θ = ${theta.toFixed(4)}`);
        console.log(`  d = vec.dot(z_curr) = ${d.toFixed(4)}`);
        console.log(`  a = vec.dot(x_prev) = ${a.toFixed(4)}`);
        console.log(`  α = ${alpha.toFixed(4)}`);
        
        // 计算 alpha: 绕x_{i-1}轴，从z_{i-1}到z_i的角度（参考Python line 800-814）
        // Project zi-1 and zi onto the plane perpendicular to xi-1
        const z_prev_proj = z_prev.clone().sub(
            x_prev.clone().multiplyScalar(z_prev.dot(x_prev))
        );
        const z_curr_proj = z_curr.clone().sub(
            x_prev.clone().multiplyScalar(z_curr.dot(x_prev))
        );
        
        // Normalize the projected vectors (参考Python line 806-807)
        const len_z_prev = z_prev_proj.length();
        const len_z_curr = z_curr_proj.length();
        
        if (len_z_prev < 1e-9 || len_z_curr < 1e-9) {
            // 投影长度接近0，alpha为0
            alpha = 0;
        } else {
            // Normalize
            const z_curr_proj_norm = z_curr_proj.clone().normalize();
            const z_prev_proj_norm = z_prev_proj.clone().normalize();
            
            // Calculate cosine and sine of the angle (参考Python line 810-811)
            const cos_alpha = z_prev_proj_norm.dot(z_curr_proj_norm);
            const sin_alpha = new THREE.Vector3().crossVectors(z_prev_proj_norm, z_curr_proj_norm).dot(x_prev);
            
            // Compute the directed angle using atan2 (参考Python line 814)
            alpha = Math.atan2(sin_alpha, cos_alpha);
        }
        
        // 清理非常小的值
        if (Math.abs(theta) < 1e-9) theta = 0;
        if (Math.abs(d) < 1e-9) d = 0;
        if (Math.abs(a) < 1e-9) a = 0;
        if (Math.abs(alpha) < 1e-9) alpha = 0;
        
        mdhParams.push([theta, d, a, alpha]);
    }
    
    // 计算最后一个关节的MDH坐标系（用于坐标系转换）
    const lastOrigin = mdhOrigins[mdhOrigins.length - 1];
    const lastX = mdhXAxes[mdhXAxes.length - 1];
    const lastZ = jointZAxes[jointZAxes.length - 1];
    const lastY = new THREE.Vector3().crossVectors(lastZ, lastX).normalize();
    
    // 重新规范化（确保正交性）
    const lastZNormalized = lastZ.clone().normalize();
    const lastXNormalized = lastX.clone().normalize();
    const lastYNormalized = lastY.clone().normalize();
    
    console.log('\n  最后一个关节的MDH坐标系:');
    console.log(`    原点: (${lastOrigin.x.toFixed(4)}, ${lastOrigin.y.toFixed(4)}, ${lastOrigin.z.toFixed(4)})`);
    console.log(`    X轴: (${lastXNormalized.x.toFixed(4)}, ${lastXNormalized.y.toFixed(4)}, ${lastXNormalized.z.toFixed(4)})`);
    console.log(`    Y轴: (${lastYNormalized.x.toFixed(4)}, ${lastYNormalized.y.toFixed(4)}, ${lastYNormalized.z.toFixed(4)})`);
    console.log(`    Z轴: (${lastZNormalized.x.toFixed(4)}, ${lastZNormalized.y.toFixed(4)}, ${lastZNormalized.z.toFixed(4)})`);
    
    // 计算所有MDH坐标系的完整信息（用于可视化）
    const allMDHFrames = [];
    console.log(`\n计算allMDHFrames: mdhOrigins.length=${mdhOrigins.length}, mdhZAxes.length=${mdhZAxes.length}`);
    
    // 添加MDH0（与base_link重合）
    allMDHFrames.push({
        origin: baseOrigin.clone(),
        xAxis: baseXAxis.clone().normalize(),
        yAxis: new THREE.Vector3().crossVectors(baseZAxis, baseXAxis).normalize(),
        zAxis: baseZAxis.clone().normalize()
    });
    console.log(`  MDH0 位置: (${baseOrigin.x.toFixed(4)}, ${baseOrigin.y.toFixed(4)}, ${baseOrigin.z.toFixed(4)})`);
    
    // 添加其他MDH坐标系（MDH1, MDH2, ...）
    for (let i = 0; i < mdhOrigins.length; i++) {
        const origin = mdhOrigins[i];
        const xAxis = mdhXAxes[i].clone().normalize();
        const zAxis = mdhZAxes[i].clone().normalize();
        const yAxis = new THREE.Vector3().crossVectors(zAxis, xAxis).normalize();
        
        allMDHFrames.push({
            origin: origin,
            xAxis: xAxis,
            yAxis: yAxis,
            zAxis: zAxis
        });
        
        console.log(`  MDH${i+1} 位置: (${origin.x.toFixed(4)}, ${origin.y.toFixed(4)}, ${origin.z.toFixed(4)})`);
    }
    
    console.log(`总共创建了 ${allMDHFrames.length} 个MDH坐标系`);
    
    // 返回MDH参数和坐标系信息
    const baseFrameInfo = {
        baseFrame: allMDHFrames.length > 0 ? allMDHFrames[0] : null,
        firstJointFrame: allMDHFrames.length > 1 ? allMDHFrames[1] : (allMDHFrames.length > 0 ? allMDHFrames[0] : null)
    };

    return {
        mdhParams: mdhParams,
        mdhFrameInfo: {
            lastOrigin: lastOrigin,
            lastX: lastXNormalized,
            lastY: lastYNormalized,
            lastZ: lastZNormalized,
            lastZAxis: lastZNormalized  // 关节的旋转轴
        },
        allMDHFrames: allMDHFrames,  // 所有MDH坐标系信息（用于可视化）
        baseFrameInfo: baseFrameInfo
    };
}

/**
 * 从 URDF 文件 URL 解析并提取 MDH 参数
 * @param {string} urdfPath - URDF 文件路径
 * @param {Array<string>} jointNames - 关节名称列表
 * @returns {Promise<Array<Array<number>>>} MDH 参数矩阵
 */
export async function parseMDHFromURDFFile(urdfPath, jointNames) {
    // 加载 URDF
    const URDFLoader = (await import('urdf-loader')).default;
    const loader = new URDFLoader();
    
    return new Promise((resolve, reject) => {
        loader.load(
            urdfPath,
            (robot) => {
                try {
                    const mdh = extractMDHFromURDF(robot, jointNames);
                    resolve(mdh);
                } catch (error) {
                    reject(error);
                }
            },
            undefined,
            (error) => reject(error)
        );
    });
}

/**
 * 高级 URDF 到 MDH 转换（更精确的算法）
 * 
 * 参考 URDFly 的实现逻辑
 * @param {Object} urdfRobot 
 * @param {Array<string>} jointNames 
 * @returns {Object} {mdh: Array, baseOffset: Object, endOffset: Object}
 */
export function extractMDHAdvanced(urdfRobot, jointNames) {
    const frames = [];
    
    // 1. 构建变换链
    for (const jointName of jointNames) {
        const joint = urdfRobot.joints[jointName];
        if (!joint) continue;
        
        // 获取关节的世界变换
        const worldMatrix = new THREE.Matrix4();
        joint.updateMatrixWorld(true);
        worldMatrix.copy(joint.matrixWorld);
        
        // 分解为位置和旋转
        const position = new THREE.Vector3();
        const quaternion = new THREE.Quaternion();
        const scale = new THREE.Vector3();
        worldMatrix.decompose(position, quaternion, scale);
        
        frames.push({
            name: jointName,
            position: position,
            quaternion: quaternion,
            axis: joint.axis ? joint.axis.clone() : new THREE.Vector3(0, 0, 1)
        });
    }
    
    // 2. 计算相邻帧之间的 MDH 参数
    const mdhParams = [];
    
    for (let i = 0; i < frames.length; i++) {
        const currentFrame = frames[i];
        const prevFrame = i > 0 ? frames[i - 1] : null;
        
        let theta = 0;
        let d = 0;
        let a = 0;
        let alpha = 0;
        
        if (prevFrame) {
            // 计算从前一帧到当前帧的变换
            const relativePos = currentFrame.position.clone().sub(prevFrame.position);
            
            // 投影到各个轴
            const prevZ = new THREE.Vector3(0, 0, 1).applyQuaternion(prevFrame.quaternion);
            const currentX = new THREE.Vector3(1, 0, 0).applyQuaternion(currentFrame.quaternion);
            
            // d: 沿前一个 Z 轴的距离
            d = relativePos.dot(prevZ);
            
            // a: 沿当前 X 轴的距离
            a = relativePos.dot(currentX);
            
            // α: 前一个Z轴到当前Z轴的角度（绕X轴）
            const currentZ = new THREE.Vector3(0, 0, 1).applyQuaternion(currentFrame.quaternion);
            const cross = new THREE.Vector3().crossVectors(prevZ, currentZ);
            const dot = prevZ.dot(currentZ);
            alpha = Math.atan2(cross.length(), dot);
            
            // 根据叉积方向确定符号
            if (cross.dot(currentX) < 0) {
                alpha = -alpha;
            }
        } else {
            // 第一个关节，相对于基座
            d = currentFrame.position.z;
            a = currentFrame.position.x;
            
            const euler = new THREE.Euler().setFromQuaternion(currentFrame.quaternion);
            alpha = euler.x;
        }
        
        mdhParams.push([theta, d, a, alpha]);
    }
    
    return {
        mdh: mdhParams,
        jointNames: jointNames,
        baseOffset: frames[0] ? frames[0].position : new THREE.Vector3(),
        numJoints: mdhParams.length
    };
}

/**
 * 格式化 MDH 参数为 DQ Robotics 可用的格式
 * @param {Array<Array<number>>} mdhParams 
 * @returns {Array<Array<number>>} 转置的矩阵 [θ列, d列, a列, α列, type列]
 */
export function formatMDHForDQ(mdhParams) {
    const numJoints = mdhParams.length;
    
    // DQ Robotics 期望的格式：5行 x N列
    // 第1行: θ (theta)
    // 第2行: d
    // 第3行: a
    // 第4行: α (alpha)
    // 第5行: 关节类型 (0=旋转, 1=移动)
    
    const theta_row = mdhParams.map(p => p[0]);
    const d_row = mdhParams.map(p => p[1]);
    const a_row = mdhParams.map(p => p[2]);
    const alpha_row = mdhParams.map(p => p[3]);
    const type_row = new Array(numJoints).fill(0); // 全部为旋转关节
    
    return [theta_row, d_row, a_row, alpha_row, type_row];
}

/**
 * 将 MDH 参数打印为易读格式
 * @param {Array<Array<number>>} mdhParams 
 */
export function printMDH(mdhParams) {
    console.log('\n╔════════════════════════════════════════════════════╗');
    console.log('║          Modified DH Parameters                    ║');
    console.log('╠════════════════════════════════════════════════════╣');
    console.log('║ Joint │    θ      │    d      │    a      │   α    ║');
    console.log('╠═══════╪═══════════╪═══════════╪═══════════╪════════╣');
    
    mdhParams.forEach((params, i) => {
        const [theta, d, a, alpha] = params;
        console.log(
            `║   ${(i + 1).toString().padStart(2)}  │ ` +
            `${theta.toFixed(4).padStart(9)} │ ` +
            `${d.toFixed(4).padStart(9)} │ ` +
            `${a.toFixed(4).padStart(9)} │ ` +
            `${alpha.toFixed(4).padStart(6)} ║`
        );
    });
    
    console.log('╚════════════════════════════════════════════════════╝\n');
}

/**
 * 比较 URDF 提取的 MDH 与预定义的 MDH
 * @param {Array<Array<number>>} urdfMDH - 从 URDF 提取的
 * @param {Array<Array<number>>} predefinedMDH - 预定义的
 * @returns {Object} 比较结果
 */
export function compareMDH(urdfMDH, predefinedMDH) {
    if (urdfMDH.length !== predefinedMDH.length) {
        return {
            match: false,
            message: `关节数量不匹配: ${urdfMDH.length} vs ${predefinedMDH.length}`
        };
    }
    
    const differences = [];
    const tolerance = 0.001; // 1mm 或 0.001 rad
    
    for (let i = 0; i < urdfMDH.length; i++) {
        for (let j = 0; j < 4; j++) {
            const diff = Math.abs(urdfMDH[i][j] - predefinedMDH[i][j]);
            if (diff > tolerance) {
                const paramNames = ['θ', 'd', 'a', 'α'];
                differences.push({
                    joint: i + 1,
                    param: paramNames[j],
                    urdf: urdfMDH[i][j],
                    predefined: predefinedMDH[i][j],
                    diff: diff
                });
            }
        }
    }
    
    return {
        match: differences.length === 0,
        differences: differences,
        message: differences.length === 0 
            ? '✓ MDH 参数完全一致' 
            : `⚠ 发现 ${differences.length} 处差异`
    };
}

/**
 * 从 URDF XML 文本解析 MDH 参数（不依赖 Three.js）
 * @param {string} urdfXML - URDF XML 文本
 * @param {Array<string>} jointNames - 关节名称
 * @returns {Array<Array<number>>} MDH 参数
 */
export function parseURDFToMDH(urdfXML, jointNames) {
    const parser = new DOMParser();
    const doc = parser.parseFromString(urdfXML, 'text/xml');
    
    const mdhParams = [];
    
    for (const jointName of jointNames) {
        // 查找关节
        const joints = doc.querySelectorAll('joint');
        let targetJoint = null;
        
        for (const joint of joints) {
            if (joint.getAttribute('name') === jointName) {
                targetJoint = joint;
                break;
            }
        }
        
        if (!targetJoint) {
            console.warn(`关节 ${jointName} 未在 URDF 中找到`);
            continue;
        }
        
        // 提取 origin 元素
        const origin = targetJoint.querySelector('origin');
        let xyz = [0, 0, 0];
        let rpy = [0, 0, 0];
        
        if (origin) {
            const xyzStr = origin.getAttribute('xyz');
            const rpyStr = origin.getAttribute('rpy');
            
            if (xyzStr) {
                xyz = xyzStr.split(/\s+/).map(parseFloat);
            }
            if (rpyStr) {
                rpy = rpyStr.split(/\s+/).map(parseFloat);
            }
        }
        
        // 提取 axis 元素
        const axisElem = targetJoint.querySelector('axis');
        let axis = [0, 0, 1]; // 默认 Z 轴
        if (axisElem) {
            const axisStr = axisElem.getAttribute('xyz');
            if (axisStr) {
                axis = axisStr.split(/\s+/).map(parseFloat);
            }
        }
        
        // 转换为 MDH 参数
        // 这是简化版本，完整的转换需要考虑整个运动链
        const theta = 0;  // 关节变量，初始为0
        const d = xyz[2]; // Z 方向平移
        const a = xyz[0]; // X 方向平移
        const alpha = rpy[0]; // 绕 X 轴旋转
        
        mdhParams.push([theta, d, a, alpha]);
    }
    
    return mdhParams;
}

/**
 * 生成可用于 DQ Robotics 的 JavaScript 数组
 * @param {Array<Array<number>>} mdhParams 
 * @returns {string} JavaScript 代码
 */
export function generateDQCode(mdhParams) {
    const formatted = formatMDHForDQ(mdhParams);
    
    let code = '// Modified DH 参数（从 URDF 提取）\n';
    code += 'const mdh_matrix = [\n';
    code += `    [${formatted[0].map(v => v.toFixed(6)).join(', ')}], // θ\n`;
    code += `    [${formatted[1].map(v => v.toFixed(6)).join(', ')}], // d\n`;
    code += `    [${formatted[2].map(v => v.toFixed(6)).join(', ')}], // a\n`;
    code += `    [${formatted[3].map(v => v.toFixed(6)).join(', ')}], // α\n`;
    code += `    [${formatted[4].map(v => v.toFixed(0)).join(', ')}]  // type (0=revolute)\n`;
    code += '];\n\n';
    code += '// 使用方法:\n';
    code += '// const robot = new SerialManipulator(mdh_matrix);\n';
    
    return code;
}

/**
 * Franka Panda 的标准 MDH 参数（来自官方规格）
 * 用于验证 URDF 提取的准确性
 */
export const FRANKA_PANDA_MDH_REFERENCE = [
    [0, 0.333, 0, 0],           // Joint 1
    [0, 0, 0, -Math.PI / 2],    // Joint 2
    [0, 0.316, 0, Math.PI / 2], // Joint 3
    [0, 0, 0.0825, Math.PI / 2],// Joint 4
    [0, 0.384, -0.0825, -Math.PI / 2], // Joint 5
    [0, 0, 0, Math.PI / 2],     // Joint 6
    [0, 0, 0.088, Math.PI / 2]  // Joint 7
];

/**
 * 创建用于测试的工具函数
 */
export function createMDHTestSuite(urdfRobot) {
    const jointNames = [
        'panda_joint1', 'panda_joint2', 'panda_joint3', 
        'panda_joint4', 'panda_joint5', 'panda_joint6', 
        'panda_joint7'
    ];
    
    console.log('═══════════════════════════════════════════');
    console.log('       URDF 到 MDH 参数提取测试');
    console.log('═══════════════════════════════════════════\n');
    
    // 提取 MDH
    const extractedMDH = extractMDHFromURDF(urdfRobot, jointNames);
    
    // 打印提取的参数
    console.log('📊 从 URDF 提取的 MDH 参数:');
    printMDH(extractedMDH);
    
    // 打印参考参数
    console.log('📚 官方参考 MDH 参数:');
    printMDH(FRANKA_PANDA_MDH_REFERENCE);
    
    // 比较
    const comparison = compareMDH(extractedMDH, FRANKA_PANDA_MDH_REFERENCE);
    console.log('🔍 比较结果:', comparison.message);
    
    if (comparison.differences && comparison.differences.length > 0) {
        console.log('\n⚠ 发现以下差异:');
        comparison.differences.forEach(diff => {
            console.log(
                `  关节${diff.joint} ${diff.param}: ` +
                `URDF=${diff.urdf.toFixed(4)}, ` +
                `参考=${diff.predefined.toFixed(4)}, ` +
                `差异=${diff.diff.toFixed(4)}`
            );
        });
    }
    
    // 生成代码
    console.log('\n📝 生成的 DQ Robotics 代码:');
    console.log(generateDQCode(extractedMDH));
    
    return {
        extracted: extractedMDH,
        reference: FRANKA_PANDA_MDH_REFERENCE,
        comparison: comparison
    };
}

