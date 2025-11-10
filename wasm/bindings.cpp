/**
 * DQ Robotics WebAssembly Bindings
 * 
 * 这个文件使用 Embind 将 DQ Robotics C++ 类绑定到 JavaScript
 */

#include <emscripten/bind.h>
#include <dqrobotics/DQ.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robots/KukaLw4Robot.h>
#include <Eigen/Dense>
#include <cmath>

using namespace emscripten;
using namespace DQ_robotics;
using namespace Eigen;

// 辅助函数：将 JavaScript 数组转换为 VectorXd
VectorXd jsArrayToVectorXd(const val& jsArray) {
    int length = jsArray["length"].as<int>();
    VectorXd vec(length);
    for (int i = 0; i < length; i++) {
        vec(i) = jsArray[i].as<double>();
    }
    return vec;
}

// 辅助函数：将 VectorXd 转换为 JavaScript 数组
val vectorXdToJsArray(const VectorXd& vec) {
    val jsArray = val::array();
    for (int i = 0; i < vec.size(); i++) {
        jsArray.call<void>("push", vec(i));
    }
    return jsArray;
}

// 辅助函数：将 MatrixXd 转换为 JavaScript 二维数组
val matrixXdToJsArray(const MatrixXd& mat) {
    val jsArray = val::array();
    for (int i = 0; i < mat.rows(); i++) {
        val row = val::array();
        for (int j = 0; j < mat.cols(); j++) {
            row.call<void>("push", mat(i, j));
        }
        jsArray.call<void>("push", row);
    }
    return jsArray;
}

// DQ 类的包装器，提供更友好的 JavaScript 接口
class DQ_Wrapper {
public:
    static DQ create(double q0 = 0, double q1 = 0, double q2 = 0, double q3 = 0,
                    double q4 = 0, double q5 = 0, double q6 = 0, double q7 = 0) {
        return DQ(q0, q1, q2, q3, q4, q5, q6, q7);
    }
    
    static DQ createFromArray(const val& jsArray) {
        VectorXd vec = jsArrayToVectorXd(jsArray);
        return DQ(vec);
    }
    
    static val toArray(const DQ& dq) {
        return vectorXdToJsArray(dq.q);
    }
    
    static DQ multiply(const DQ& dq1, const DQ& dq2) {
        return dq1 * dq2;
    }
    
    static DQ add(const DQ& dq1, const DQ& dq2) {
        return dq1 + dq2;
    }
    
    static DQ conjugate(const DQ& dq) {
        return dq.conj();
    }
    
    static DQ normalize(const DQ& dq) {
        return dq.normalize();
    }
    
    static DQ inverse(const DQ& dq) {
        return dq.inv();
    }
    
    static DQ rotation(double angle, double x, double y, double z) {
        // 归一化旋转轴
        double norm = std::sqrt(x*x + y*y + z*z);
        if (norm > 1e-10) {
            x /= norm; y /= norm; z /= norm;
        }
        // 创建单位四元数表示旋转
        double half_angle = angle / 2.0;
        double s = std::sin(half_angle);
        double c = std::cos(half_angle);
        return DQ(c, s*x, s*y, s*z);
    }
    
    static DQ translation(double x, double y, double z) {
        return DQ(1) + 0.5 * DQ::E * DQ(0, x, y, z);
    }
    
    static DQ pose(double rx, double ry, double rz, double angle, 
                   double tx, double ty, double tz) {
        // 旋转
        double norm = std::sqrt(rx*rx + ry*ry + rz*rz);
        if (norm > 1e-10) {
            rx /= norm; ry /= norm; rz /= norm;
        }
        double half_angle = angle / 2.0;
        double s = std::sin(half_angle);
        double c = std::cos(half_angle);
        DQ r = DQ(c, s*rx, s*ry, s*rz);
        
        // 平移
        DQ t = DQ(1) + 0.5 * DQ::E * DQ(0, tx, ty, tz);
        return r * t;
    }
    
    static val getTranslation(const DQ& dq) {
        DQ t = ::translation(dq);  // 使用全局函数
        val result = val::array();
        result.call<void>("push", 0.0);  // w 分量
        result.call<void>("push", t.q(1));  // x
        result.call<void>("push", t.q(2));  // y
        result.call<void>("push", t.q(3));  // z
        return result;
    }
    
    static val getRotation(const DQ& dq) {
        DQ r = ::rotation(dq);  // 使用全局函数
        return vectorXdToJsArray(r.q);
    }
    
    static val getRotationMatrix(const DQ& dq) {
        // 从旋转四元数计算旋转矩阵
        DQ r = ::rotation(dq);
        double w = r.q(0), x = r.q(1), y = r.q(2), z = r.q(3);
        
        MatrixXd mat(3, 3);
        mat(0,0) = 1 - 2*(y*y + z*z);
        mat(0,1) = 2*(x*y - w*z);
        mat(0,2) = 2*(x*z + w*y);
        mat(1,0) = 2*(x*y + w*z);
        mat(1,1) = 1 - 2*(x*x + z*z);
        mat(1,2) = 2*(y*z - w*x);
        mat(2,0) = 2*(x*z - w*y);
        mat(2,1) = 2*(y*z + w*x);
        mat(2,2) = 1 - 2*(x*x + y*y);
        
        return matrixXdToJsArray(mat);
    }
};

// 串联机械臂包装器（使用 Modified DH）
class SerialManipulator_Wrapper {
private:
    DQ_SerialManipulatorMDH robot;
    
    static MatrixXd extractDHMatrix(const val& dh_matrix_js) {
        // JavaScript 传入的格式: [[θ1,θ2,...], [d1,d2,...], [a1,a2,...], [α1,α2,...], [type1,type2,...]]
        // 即: 5行 x n列
        int num_rows = dh_matrix_js["length"].as<int>();
        if (num_rows != 5) {
            throw std::runtime_error("DH matrix should have 5 rows");
        }
        
        val first_row = dh_matrix_js[0];
        int num_joints = first_row["length"].as<int>();
        
        MatrixXd dh_matrix(5, num_joints);
        for (int row = 0; row < 5; row++) {
            val js_row = dh_matrix_js[row];
            for (int col = 0; col < num_joints; col++) {
                dh_matrix(row, col) = js_row[col].as<double>();
            }
        }
        return dh_matrix;
    }
    
public:
    SerialManipulator_Wrapper(const val& dh_matrix_js) 
        : robot(extractDHMatrix(dh_matrix_js)) {
    }
    
    val fkm(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        DQ pose = robot.fkm(joint_angles);
        return vectorXdToJsArray(pose.q);
    }
    
    val pose_jacobian(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        MatrixXd J = robot.pose_jacobian(joint_angles);
        return matrixXdToJsArray(J);
    }
    
    int get_dim_configuration_space() const {
        return robot.get_dim_configuration_space();
    }
    
    // 设置基座偏移
    void set_base_frame(double x, double y, double z) {
        DQ base = 1 + E_ * 0.5 * DQ(0, x, y, z);
        base = base.normalize(); // 归一化（与Python逻辑一致）
        robot.set_base_frame(base);
        robot.set_reference_frame(base);
    }
    
    // 设置基座完整姿态（旋转 + 平移）
    void set_base_frame_pose(double qw, double qx, double qy, double qz,
                             double tx, double ty, double tz) {
        // 先归一化旋转四元数（只归一化旋转部分，不改变平移）
        double q_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (q_norm > 1e-10) {
            qw /= q_norm;
            qx /= q_norm;
            qy /= q_norm;
            qz /= q_norm;
        }
        // 直接构造双四元数：前4个是旋转四元数，后4个是1/2*平移*旋转四元数
        // DQ格式：[qw, qx, qy, qz, qw_dual, qx_dual, qy_dual, qz_dual]
        // 其中对偶部分 = 0.5 * [0, tx, ty, tz] * [qw, qx, qy, qz]
        // 对偶四元数乘法：(0, tx, ty, tz) * (qw, qx, qy, qz) = (0, tx, ty, tz) * 四元数乘法
        // 纯四元数 * 四元数：(0, tx, ty, tz) * (qw, qx, qy, qz)
        // = (0*qw - (tx,ty,tz)·(qx,qy,qz), 0*(qx,qy,qz) + qw*(tx,ty,tz) + (tx,ty,tz)×(qx,qy,qz))
        // = (-tx*qx - ty*qy - tz*qz, qw*tx + ty*qz - tz*qy, qw*ty + tz*qx - tx*qz, qw*tz + tx*qy - ty*qx)
        double dual_w = -0.5 * (tx*qx + ty*qy + tz*qz);
        double dual_x = 0.5 * (qw*tx + ty*qz - tz*qy);
        double dual_y = 0.5 * (qw*ty + tz*qx - tx*qz);
        double dual_z = 0.5 * (qw*tz + tx*qy - ty*qx);
        
        // 直接构造8元素双四元数
        DQ base(qw, qx, qy, qz, dual_w, dual_x, dual_y, dual_z);
        robot.set_base_frame(base);
        robot.set_reference_frame(base);
    }
    
    // 设置基座（直接使用双四元数，8元素数组）
    void set_base_frame_dq(const val& dq_array) {
        VectorXd dq_vec = jsArrayToVectorXd(dq_array);
        if (dq_vec.size() != 8) {
            throw std::runtime_error("双四元数必须是8元素数组");
        }
        DQ base(dq_vec);
        
        // 打印原始双四元数（用于调试）
        printf("原始双四元数: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               dq_vec(0), dq_vec(1), dq_vec(2), dq_vec(3),
               dq_vec(4), dq_vec(5), dq_vec(6), dq_vec(7));
        
        // 归一化（与Python代码保持一致）
        base = base.normalize();
        
        // 打印归一化后的双四元数（用于调试）
        printf("归一化后: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               base.q(0), base.q(1), base.q(2), base.q(3),
               base.q(4), base.q(5), base.q(6), base.q(7));
        
        robot.set_base_frame(base);
        robot.set_reference_frame(base);
    }
    
    // 设置末端偏移
    void set_effector(double x, double y, double z) {
        DQ eff = 1 + E_ * 0.5 * DQ(0, x, y, z);
        robot.set_effector(eff);
    }

    // 设置末端完整姿态（旋转 + 平移）
    void set_effector_pose(double qw, double qx, double qy, double qz,
                           double tx, double ty, double tz) {
        DQ r(qw, qx, qy, qz);
        DQ t = 1 + E_ * 0.5 * DQ(0, tx, ty, tz);
        DQ eff = r * t; // 先旋转再平移（工具坐标）
        robot.set_effector(eff);
    }
};

// 预定义机器人包装器 - Franka Panda (使用 MDH 参数)
class FrankaPanda_Wrapper {
private:
    DQ_SerialManipulatorMDH robot;
    
public:
    FrankaPanda_Wrapper() : robot(FrankaEmikaPandaRobot::kinematics()) {
    }
    
    val fkm(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        DQ pose = robot.fkm(joint_angles);
        return vectorXdToJsArray(pose.q);
    }
    
    val pose_jacobian(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        MatrixXd J = robot.pose_jacobian(joint_angles);
        return matrixXdToJsArray(J);
    }
    
    val get_end_effector_pose_array(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        DQ pose = robot.fkm(joint_angles);
        
        // 获取平移和旋转
        DQ trans = ::translation(pose);
        DQ rot = ::rotation(pose);
        
        val translation_array = val::array();
        translation_array.call<void>("push", 0.0);  // w 分量
        translation_array.call<void>("push", trans.q(1));  // x
        translation_array.call<void>("push", trans.q(2));  // y
        translation_array.call<void>("push", trans.q(3));  // z
        
        val result = val::object();
        result.set("translation", translation_array);
        result.set("rotation", vectorXdToJsArray(rot.q));
        
        return result;
    }
    
    int get_dim_configuration_space() const {
        return robot.get_dim_configuration_space();
    }
};

// Kuka LW4 机器人 (使用标准 DH 参数)
class KukaLw4_Wrapper {
private:
    DQ_SerialManipulatorDH robot;
    
public:
    KukaLw4_Wrapper() : robot(KukaLw4Robot::kinematics()) {
    }
    
    val fkm(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        DQ pose = robot.fkm(joint_angles);
        return vectorXdToJsArray(pose.q);
    }
    
    val get_end_effector_pose_array(const val& joint_angles_js) {
        VectorXd joint_angles = jsArrayToVectorXd(joint_angles_js);
        DQ pose = robot.fkm(joint_angles);
        
        // 获取平移和旋转
        DQ trans = ::translation(pose);
        DQ rot = ::rotation(pose);
        
        val translation_array = val::array();
        translation_array.call<void>("push", 0.0);  // w 分量
        translation_array.call<void>("push", trans.q(1));  // x
        translation_array.call<void>("push", trans.q(2));  // y
        translation_array.call<void>("push", trans.q(3));  // z
        
        val result = val::object();
        result.set("translation", translation_array);
        result.set("rotation", vectorXdToJsArray(rot.q));
        
        return result;
    }
    
    int get_dim_configuration_space() const {
        return robot.get_dim_configuration_space();
    }
};

// 协作双臂系统包装器
class CooperativeDualTaskSpace_Wrapper {
private:
    DQ_CooperativeDualTaskSpace* dual_arm;
    DQ_SerialManipulatorMDH* robot1_ptr;
    DQ_SerialManipulatorMDH* robot2_ptr;
    bool owns_robots;
    
public:
    // 从两个已存在的机器人指针创建（不拥有所有权）
    CooperativeDualTaskSpace_Wrapper(SerialManipulator_Wrapper* r1, SerialManipulator_Wrapper* r2) 
        : owns_robots(false) {
        // 注意：这里需要访问内部robot，但embind限制，所以需要另一种方式
        // 为了简化，我们创建新的机器人实例
        throw std::runtime_error("Use createFromDHMatrices instead");
    }
    
    // 从DH矩阵创建（会创建新的机器人实例）
    CooperativeDualTaskSpace_Wrapper(const val& robot1_dh_js, const val& robot2_dh_js) 
        : owns_robots(true) {
        robot1_ptr = new DQ_SerialManipulatorMDH(extractDHMatrixStatic(robot1_dh_js));
        robot2_ptr = new DQ_SerialManipulatorMDH(extractDHMatrixStatic(robot2_dh_js));
        dual_arm = new DQ_CooperativeDualTaskSpace(robot1_ptr, robot2_ptr);
    }
    
    ~CooperativeDualTaskSpace_Wrapper() {
        if (owns_robots) {
            delete dual_arm;
            delete robot1_ptr;
            delete robot2_ptr;
        }
    }
    
    static MatrixXd extractDHMatrixStatic(const val& dh_matrix_js) {
        int num_rows = dh_matrix_js["length"].as<int>();
        if (num_rows != 5) {
            throw std::runtime_error("DH matrix should have 5 rows");
        }
        
        val first_row = dh_matrix_js[0];
        int num_joints = first_row["length"].as<int>();
        
        MatrixXd dh_matrix(5, num_joints);
        for (int row = 0; row < 5; row++) {
            val js_row = dh_matrix_js[row];
            for (int col = 0; col < num_joints; col++) {
                dh_matrix(row, col) = js_row[col].as<double>();
            }
        }
        return dh_matrix;
    }
    
    // 设置机器人1的基座和末端
    void setRobot1Base(double x, double y, double z) {
        DQ base = 1 + E_ * 0.5 * DQ(0, x, y, z);
        base = base.normalize(); // 归一化（与Python逻辑一致）
        robot1_ptr->set_base_frame(base);
        robot1_ptr->set_reference_frame(base);
    }
    
    void setRobot1BasePose(double qw, double qx, double qy, double qz,
                           double tx, double ty, double tz) {
        // 先归一化旋转四元数（只归一化旋转部分，不改变平移）
        double q_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (q_norm > 1e-10) {
            qw /= q_norm;
            qx /= q_norm;
            qy /= q_norm;
            qz /= q_norm;
        }
        DQ r(qw, qx, qy, qz);
        DQ t = 1 + E_ * 0.5 * DQ(0, tx, ty, tz);
        DQ base = r * t; // 先旋转再平移
        robot1_ptr->set_base_frame(base);
        robot1_ptr->set_reference_frame(base);
    }
    
    // 设置机器人1基座（直接使用双四元数，8元素数组）
    void setRobot1BaseDQ(const val& dq_array) {
        VectorXd dq_vec = jsArrayToVectorXd(dq_array);
        if (dq_vec.size() != 8) {
            throw std::runtime_error("双四元数必须是8元素数组");
        }
        DQ base(dq_vec);
        
        // 打印原始双四元数（用于调试）
        printf("机器人1原始双四元数: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               dq_vec(0), dq_vec(1), dq_vec(2), dq_vec(3),
               dq_vec(4), dq_vec(5), dq_vec(6), dq_vec(7));
        
        // 归一化（与Python代码保持一致）
        base = base.normalize();
        
        // 打印归一化后的双四元数（用于调试）
        printf("机器人1归一化后: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               base.q(0), base.q(1), base.q(2), base.q(3),
               base.q(4), base.q(5), base.q(6), base.q(7));
        
        robot1_ptr->set_base_frame(base);
        robot1_ptr->set_reference_frame(base);
    }
    
    void setRobot1Effector(double x, double y, double z) {
        DQ eff = 1 + E_ * 0.5 * DQ(0, x, y, z);
        robot1_ptr->set_effector(eff);
    }
    
    void setRobot1EffectorPose(double qw, double qx, double qy, double qz,
                                double tx, double ty, double tz) {
        DQ r(qw, qx, qy, qz);
        DQ t = 1 + E_ * 0.5 * DQ(0, tx, ty, tz);
        robot1_ptr->set_effector(r * t);
    }
    
    // 设置机器人2的基座和末端
    void setRobot2Base(double x, double y, double z) {
        DQ base = 1 + E_ * 0.5 * DQ(0, x, y, z);
        base = base.normalize(); // 归一化（与Python逻辑一致）
        robot2_ptr->set_base_frame(base);
        robot2_ptr->set_reference_frame(base);
    }
    
    void setRobot2BasePose(double qw, double qx, double qy, double qz,
                           double tx, double ty, double tz) {
        // 先归一化旋转四元数（只归一化旋转部分，不改变平移）
        double q_norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (q_norm > 1e-10) {
            qw /= q_norm;
            qx /= q_norm;
            qy /= q_norm;
            qz /= q_norm;
        }
        DQ r(qw, qx, qy, qz);
        DQ t = 1 + E_ * 0.5 * DQ(0, tx, ty, tz);
        DQ base = r * t; // 先旋转再平移
        robot2_ptr->set_base_frame(base);
        robot2_ptr->set_reference_frame(base);
    }
    
    // 设置机器人2基座（直接使用双四元数，8元素数组）
    void setRobot2BaseDQ(const val& dq_array) {
        VectorXd dq_vec = jsArrayToVectorXd(dq_array);
        if (dq_vec.size() != 8) {
            throw std::runtime_error("双四元数必须是8元素数组");
        }
        DQ base(dq_vec);
        
        // 打印原始双四元数（用于调试）
        printf("机器人2原始双四元数: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               dq_vec(0), dq_vec(1), dq_vec(2), dq_vec(3),
               dq_vec(4), dq_vec(5), dq_vec(6), dq_vec(7));
        
        // 归一化（与Python代码保持一致）
        base = base.normalize();
        
        // 打印归一化后的双四元数（用于调试）
        printf("机器人2归一化后: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
               base.q(0), base.q(1), base.q(2), base.q(3),
               base.q(4), base.q(5), base.q(6), base.q(7));
        
        robot2_ptr->set_base_frame(base);
        robot2_ptr->set_reference_frame(base);
    }
    
    void setRobot2Effector(double x, double y, double z) {
        DQ eff = 1 + E_ * 0.5 * DQ(0, x, y, z);
        robot2_ptr->set_effector(eff);
    }
    
    void setRobot2EffectorPose(double qw, double qx, double qy, double qz,
                                double tx, double ty, double tz) {
        DQ r(qw, qx, qy, qz);
        DQ t = 1 + E_ * 0.5 * DQ(0, tx, ty, tz);
        robot2_ptr->set_effector(r * t);
    }
    
    // 获取机器人1的位姿
    val pose1(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        DQ pose = dual_arm->pose1(theta);
        return vectorXdToJsArray(pose.q);
    }
    
    // 获取机器人2的位姿
    val pose2(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        DQ pose = dual_arm->pose2(theta);
        return vectorXdToJsArray(pose.q);
    }
    
    // 获取相对位姿（robot2相对于robot1）
    val relative_pose(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        DQ pose = dual_arm->relative_pose(theta);
        return vectorXdToJsArray(pose.q);
    }
    
    // 获取绝对位姿（两个末端的平均位置）
    val absolute_pose(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        DQ pose = dual_arm->absolute_pose(theta);
        return vectorXdToJsArray(pose.q);
    }
    
    // 获取相对位姿雅可比
    val relative_pose_jacobian(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        MatrixXd J = dual_arm->relative_pose_jacobian(theta);
        return matrixXdToJsArray(J);
    }
    
    // 获取绝对位姿雅可比
    val absolute_pose_jacobian(const val& joint_angles_js) {
        VectorXd theta = jsArrayToVectorXd(joint_angles_js);
        MatrixXd J = dual_arm->absolute_pose_jacobian(theta);
        return matrixXdToJsArray(J);
    }
};

// Embind 绑定
EMSCRIPTEN_BINDINGS(dqrobotics_module) {
    // DQ 类
    class_<DQ>("DQ")
        .constructor<>()
        .constructor<double, double, double, double, double, double, double, double>()
        .property("q", &DQ::q);
    
    // DQ 包装器 - 提供静态方法
    class_<DQ_Wrapper>("DQWrapper")
        .class_function("create", &DQ_Wrapper::create)
        .class_function("createFromArray", &DQ_Wrapper::createFromArray)
        .class_function("toArray", &DQ_Wrapper::toArray)
        .class_function("multiply", &DQ_Wrapper::multiply)
        .class_function("add", &DQ_Wrapper::add)
        .class_function("conjugate", &DQ_Wrapper::conjugate)
        .class_function("normalize", &DQ_Wrapper::normalize)
        .class_function("inverse", &DQ_Wrapper::inverse)
        .class_function("rotation", &DQ_Wrapper::rotation)
        .class_function("translation", &DQ_Wrapper::translation)
        .class_function("pose", &DQ_Wrapper::pose)
        .class_function("getTranslation", &DQ_Wrapper::getTranslation)
        .class_function("getRotation", &DQ_Wrapper::getRotation)
        .class_function("getRotationMatrix", &DQ_Wrapper::getRotationMatrix);
    
    // 串联机械臂
    class_<SerialManipulator_Wrapper>("SerialManipulator")
        .constructor<const val&>()
        .function("fkm", &SerialManipulator_Wrapper::fkm)
        .function("poseJacobian", &SerialManipulator_Wrapper::pose_jacobian)
        .function("getDimConfigurationSpace", &SerialManipulator_Wrapper::get_dim_configuration_space)
        .function("setBaseFrame", &SerialManipulator_Wrapper::set_base_frame)
        .function("setBaseFramePose", &SerialManipulator_Wrapper::set_base_frame_pose)
        .function("setBaseFrameDQ", &SerialManipulator_Wrapper::set_base_frame_dq)
        .function("setEffector", &SerialManipulator_Wrapper::set_effector)
        .function("setEffectorPose", &SerialManipulator_Wrapper::set_effector_pose);
    
    // Franka Panda 机器人
    class_<FrankaPanda_Wrapper>("FrankaPanda")
        .constructor<>()
        .function("fkm", &FrankaPanda_Wrapper::fkm)
        .function("poseJacobian", &FrankaPanda_Wrapper::pose_jacobian)
        .function("getEndEffectorPose", &FrankaPanda_Wrapper::get_end_effector_pose_array)
        .function("getDimConfigurationSpace", &FrankaPanda_Wrapper::get_dim_configuration_space);
    
    // Kuka LW4 机器人
    class_<KukaLw4_Wrapper>("KukaLw4")
        .constructor<>()
        .function("fkm", &KukaLw4_Wrapper::fkm)
        .function("getEndEffectorPose", &KukaLw4_Wrapper::get_end_effector_pose_array)
        .function("getDimConfigurationSpace", &KukaLw4_Wrapper::get_dim_configuration_space);
    
    // 协作双臂系统
    class_<CooperativeDualTaskSpace_Wrapper>("CooperativeDualTaskSpace")
        .constructor<const val&, const val&>()
        .function("setRobot1Base", &CooperativeDualTaskSpace_Wrapper::setRobot1Base)
        .function("setRobot1BasePose", &CooperativeDualTaskSpace_Wrapper::setRobot1BasePose)
        .function("setRobot1BaseDQ", &CooperativeDualTaskSpace_Wrapper::setRobot1BaseDQ)
        .function("setRobot1Effector", &CooperativeDualTaskSpace_Wrapper::setRobot1Effector)
        .function("setRobot1EffectorPose", &CooperativeDualTaskSpace_Wrapper::setRobot1EffectorPose)
        .function("setRobot2Base", &CooperativeDualTaskSpace_Wrapper::setRobot2Base)
        .function("setRobot2BasePose", &CooperativeDualTaskSpace_Wrapper::setRobot2BasePose)
        .function("setRobot2BaseDQ", &CooperativeDualTaskSpace_Wrapper::setRobot2BaseDQ)
        .function("setRobot2Effector", &CooperativeDualTaskSpace_Wrapper::setRobot2Effector)
        .function("setRobot2EffectorPose", &CooperativeDualTaskSpace_Wrapper::setRobot2EffectorPose)
        .function("pose1", &CooperativeDualTaskSpace_Wrapper::pose1)
        .function("pose2", &CooperativeDualTaskSpace_Wrapper::pose2)
        .function("relativePose", &CooperativeDualTaskSpace_Wrapper::relative_pose)
        .function("absolutePose", &CooperativeDualTaskSpace_Wrapper::absolute_pose)
        .function("relativePoseJacobian", &CooperativeDualTaskSpace_Wrapper::relative_pose_jacobian)
        .function("absolutePoseJacobian", &CooperativeDualTaskSpace_Wrapper::absolute_pose_jacobian);
}

