/**
 * 机械臂逆运动学解算模块
 * 支持CCD和雅可比矩阵迭代法
 */

const RobotArmIK = (function() {
    'use strict';

    // ========================================
    // 配置
    // ========================================
    
    const config = {
        maxIterations: 100,      // 最大迭代次数
        positionTolerance: 0.5,  // 位置误差容限 (mm)
        orientationTolerance: 0.01, // 姿态误差容限 (rad)
        dampingFactor: 0.5,      // 阻尼因子（用于雅可比法）
        stepSize: 0.1            // CCD步长限制
    };

    // ========================================
    // 工具函数
    // ========================================

    /**
     * 构建关节的基础变换矩阵
     */
    function buildBaseTransform(joint) {
        const T = new THREE.Matrix4();
        const trans = new THREE.Matrix4().makeTranslation(joint.x, joint.y, joint.z);
        const euler = new THREE.Euler(
            joint.rx * Math.PI / 180,
            joint.ry * Math.PI / 180,
            joint.rz * Math.PI / 180,
            'ZYX'
        );
        const rot = new THREE.Matrix4().makeRotationFromEuler(euler);
        T.multiply(trans).multiply(rot);
        return T;
    }

    /**
     * 构建关节变量变换
     */
    function buildJointTransform(type, value) {
        if (type === 'R') {
            return new THREE.Matrix4().makeRotationZ(value);
        } else {
            return new THREE.Matrix4().makeTranslation(0, 0, value);
        }
    }

    /**
     * 计算完整FK，返回所有关节位置和末端位姿
     */
    function computeFullFK(joints, jointValues) {
        const positions = [new THREE.Vector3(0, 0, 0)];
        const orientations = [new THREE.Quaternion()];
        let T = new THREE.Matrix4();

        for (let i = 0; i < joints.length; i++) {
            const T_base = buildBaseTransform(joints[i]);
            const T_joint = buildJointTransform(joints[i].type, jointValues[i]);
            T = T.clone().multiply(T_base).multiply(T_joint);

            const pos = new THREE.Vector3();
            pos.setFromMatrixPosition(T);
            positions.push(pos);

            const quat = new THREE.Quaternion();
            quat.setFromRotationMatrix(T);
            orientations.push(quat);
        }

        return {
            positions,
            orientations,
            endPosition: positions[positions.length - 1].clone(),
            endOrientation: orientations[orientations.length - 1].clone(),
            endTransform: T.clone()
        };
    }

    /**
     * 计算末端位置（快速版本）
     */
    function computeEndPosition(joints, jointValues) {
        let T = new THREE.Matrix4();
        for (let i = 0; i < joints.length; i++) {
            const T_base = buildBaseTransform(joints[i]);
            const T_joint = buildJointTransform(joints[i].type, jointValues[i]);
            T = T.clone().multiply(T_base).multiply(T_joint);
        }
        const pos = new THREE.Vector3();
        pos.setFromMatrixPosition(T);
        return pos;
    }

    /**
     * 计算末端位姿
     */
    function computeEndPose(joints, jointValues) {
        let T = new THREE.Matrix4();
        for (let i = 0; i < joints.length; i++) {
            const T_base = buildBaseTransform(joints[i]);
            const T_joint = buildJointTransform(joints[i].type, jointValues[i]);
            T = T.clone().multiply(T_base).multiply(T_joint);
        }
        
        const pos = new THREE.Vector3();
        pos.setFromMatrixPosition(T);
        
        const quat = new THREE.Quaternion();
        quat.setFromRotationMatrix(T);
        
        return { position: pos, orientation: quat, transform: T };
    }

    // ========================================
    // CCD (Cyclic Coordinate Descent) 算法
    // ========================================

    /**
     * CCD逆运动学求解 - 仅位置
     * @param {Array} joints - 关节配置
     * @param {Array} currentValues - 当前关节值
     * @param {THREE.Vector3} targetPos - 目标位置
     * @returns {Object} - { values: 新关节值, success: 是否收敛, error: 最终误差 }
     */
    function solveCCDPosition(joints, currentValues, targetPos) {
        if (joints.length === 0) {
            return { values: [], success: false, error: Infinity };
        }

        const values = [...currentValues];
        let lastError = Infinity;

        for (let iter = 0; iter < config.maxIterations; iter++) {
            // 从末端关节向基座遍历
            for (let i = joints.length - 1; i >= 0; i--) {
                const joint = joints[i];
                
                if (joint.type === 'P') {
                    // 平动关节 - 沿关节轴方向移动
                    const fk = computeFullFK(joints, values);
                    const jointPos = fk.positions[i + 1]; // 关节i+1的位置
                    const endPos = fk.endPosition;
                    
                    // 获取关节轴方向（局部Z轴在世界坐标系中的方向）
                    let T = new THREE.Matrix4();
                    for (let j = 0; j <= i; j++) {
                        const T_base = buildBaseTransform(joints[j]);
                        const T_joint = buildJointTransform(joints[j].type, values[j]);
                        T = T.clone().multiply(T_base).multiply(T_joint);
                    }
                    const zAxis = new THREE.Vector3(0, 0, 1);
                    zAxis.applyMatrix4(T.clone().setPosition(0, 0, 0));
                    
                    // 计算目标在关节轴上的投影
                    const toTarget = new THREE.Vector3().subVectors(targetPos, endPos);
                    const delta = toTarget.dot(zAxis);
                    
                    values[i] += delta * config.stepSize;
                    // 限制范围 - 使用关节配置的限位
                    const minLimit = joint.min !== undefined ? joint.min : -500;
                    const maxLimit = joint.max !== undefined ? joint.max : 500;
                    values[i] = Math.max(minLimit, Math.min(maxLimit, values[i]));
                    
                } else {
                    // 转动关节 - 旋转使末端靠近目标
                    const fk = computeFullFK(joints, values);
                    const jointPos = fk.positions[i]; // 关节i的位置（旋转中心）
                    const endPos = fk.endPosition;
                    
                    // 获取关节轴方向
                    let T = new THREE.Matrix4();
                    for (let j = 0; j < i; j++) {
                        const T_base = buildBaseTransform(joints[j]);
                        const T_joint = buildJointTransform(joints[j].type, values[j]);
                        T = T.clone().multiply(T_base).multiply(T_joint);
                    }
                    if (i > 0) {
                        T = T.clone().multiply(buildBaseTransform(joints[i]));
                    }
                    
                    const zAxis = new THREE.Vector3(0, 0, 1);
                    if (i > 0) {
                        zAxis.applyMatrix4(T.clone().setPosition(0, 0, 0));
                    }
                    
                    // 计算从关节到末端和从关节到目标的向量
                    const toEnd = new THREE.Vector3().subVectors(endPos, jointPos);
                    const toTarget = new THREE.Vector3().subVectors(targetPos, jointPos);
                    
                    // 投影到垂直于关节轴的平面
                    const toEndProj = toEnd.clone().sub(zAxis.clone().multiplyScalar(toEnd.dot(zAxis)));
                    const toTargetProj = toTarget.clone().sub(zAxis.clone().multiplyScalar(toTarget.dot(zAxis)));
                    
                    if (toEndProj.length() < 0.001 || toTargetProj.length() < 0.001) continue;
                    
                    toEndProj.normalize();
                    toTargetProj.normalize();
                    
                    // 计算旋转角度
                    let angle = Math.acos(Math.max(-1, Math.min(1, toEndProj.dot(toTargetProj))));
                    
                    // 确定旋转方向
                    const cross = new THREE.Vector3().crossVectors(toEndProj, toTargetProj);
                    if (cross.dot(zAxis) < 0) {
                        angle = -angle;
                    }
                    
                    // 限制单步旋转角度
                    angle = Math.max(-0.5, Math.min(0.5, angle));
                    
                    values[i] += angle;
                    // 限制范围 - 使用关节配置的限位（转换为弧度）
                    const minLimit = joint.min !== undefined ? joint.min * Math.PI / 180 : -Math.PI;
                    const maxLimit = joint.max !== undefined ? joint.max * Math.PI / 180 : Math.PI;
                    values[i] = Math.max(minLimit, Math.min(maxLimit, values[i]));
                }
            }

            // 计算误差
            const currentEnd = computeEndPosition(joints, values);
            const error = currentEnd.distanceTo(targetPos);
            
            if (error < config.positionTolerance) {
                return { values, success: true, error, iterations: iter + 1 };
            }
            
            // 检查收敛
            if (Math.abs(error - lastError) < 0.001) {
                return { values, success: false, error, iterations: iter + 1 };
            }
            lastError = error;
        }

        const finalError = computeEndPosition(joints, values).distanceTo(targetPos);
        return { values, success: false, error: finalError, iterations: config.maxIterations };
    }

    /**
     * CCD逆运动学求解 - 位置和姿态
     * @param {Array} joints - 关节配置
     * @param {Array} currentValues - 当前关节值
     * @param {THREE.Vector3} targetPos - 目标位置
     * @param {THREE.Quaternion} targetOri - 目标姿态
     * @param {Number} posWeight - 位置权重 (0-1)
     * @returns {Object}
     */
    function solveCCDPose(joints, currentValues, targetPos, targetOri, posWeight = 0.7) {
        if (joints.length === 0) {
            return { values: [], success: false, error: Infinity };
        }

        const values = [...currentValues];
        const oriWeight = 1 - posWeight;

        for (let iter = 0; iter < config.maxIterations; iter++) {
            // 从末端向基座遍历
            for (let i = joints.length - 1; i >= 0; i--) {
                const joint = joints[i];
                
                if (joint.type === 'R') {
                    const fk = computeFullFK(joints, values);
                    const jointPos = fk.positions[i];
                    const endPos = fk.endPosition;
                    const endOri = fk.endOrientation;
                    
                    // 获取关节轴
                    let T = new THREE.Matrix4();
                    for (let j = 0; j < i; j++) {
                        const T_base = buildBaseTransform(joints[j]);
                        const T_joint = buildJointTransform(joints[j].type, values[j]);
                        T = T.clone().multiply(T_base).multiply(T_joint);
                    }
                    if (i > 0) {
                        T = T.clone().multiply(buildBaseTransform(joints[i]));
                    }
                    
                    const zAxis = new THREE.Vector3(0, 0, 1);
                    if (i > 0) {
                        zAxis.applyMatrix4(T.clone().setPosition(0, 0, 0));
                    }
                    
                    // 位置误差贡献
                    let anglePos = 0;
                    const toEnd = new THREE.Vector3().subVectors(endPos, jointPos);
                    const toTarget = new THREE.Vector3().subVectors(targetPos, jointPos);
                    
                    const toEndProj = toEnd.clone().sub(zAxis.clone().multiplyScalar(toEnd.dot(zAxis)));
                    const toTargetProj = toTarget.clone().sub(zAxis.clone().multiplyScalar(toTarget.dot(zAxis)));
                    
                    if (toEndProj.length() > 0.001 && toTargetProj.length() > 0.001) {
                        toEndProj.normalize();
                        toTargetProj.normalize();
                        anglePos = Math.acos(Math.max(-1, Math.min(1, toEndProj.dot(toTargetProj))));
                        const cross = new THREE.Vector3().crossVectors(toEndProj, toTargetProj);
                        if (cross.dot(zAxis) < 0) anglePos = -anglePos;
                    }
                    
                    // 姿态误差贡献
                    let angleOri = 0;
                    const oriError = endOri.clone().invert().multiply(targetOri);
                    const euler = new THREE.Euler().setFromQuaternion(oriError, 'ZYX');
                    // 提取绕关节轴的旋转分量（近似）
                    angleOri = euler.z;
                    
                    // 加权组合
                    const angle = posWeight * anglePos + oriWeight * angleOri;
                    const clampedAngle = Math.max(-0.3, Math.min(0.3, angle));
                    
                    values[i] += clampedAngle;
                    // 限制范围 - 使用关节配置的限位（转换为弧度）
                    const minLimit = joint.min !== undefined ? joint.min * Math.PI / 180 : -Math.PI;
                    const maxLimit = joint.max !== undefined ? joint.max * Math.PI / 180 : Math.PI;
                    values[i] = Math.max(minLimit, Math.min(maxLimit, values[i]));
                    
                } else {
                    // 平动关节只影响位置
                    const fk = computeFullFK(joints, values);
                    const endPos = fk.endPosition;
                    
                    let T = new THREE.Matrix4();
                    for (let j = 0; j <= i; j++) {
                        const T_base = buildBaseTransform(joints[j]);
                        const T_joint = buildJointTransform(joints[j].type, values[j]);
                        T = T.clone().multiply(T_base).multiply(T_joint);
                    }
                    const zAxis = new THREE.Vector3(0, 0, 1);
                    zAxis.applyMatrix4(T.clone().setPosition(0, 0, 0));
                    
                    const toTarget = new THREE.Vector3().subVectors(targetPos, endPos);
                    const delta = toTarget.dot(zAxis) * config.stepSize;
                    
                    values[i] += delta;
                    // 限制范围 - 使用关节配置的限位
                    const minLimit = joint.min !== undefined ? joint.min : -500;
                    const maxLimit = joint.max !== undefined ? joint.max : 500;
                    values[i] = Math.max(minLimit, Math.min(maxLimit, values[i]));
                }
            }

            // 检查收敛
            const pose = computeEndPose(joints, values);
            const posError = pose.position.distanceTo(targetPos);
            const oriError = pose.orientation.angleTo(targetOri);
            
            if (posError < config.positionTolerance && oriError < config.orientationTolerance) {
                return { 
                    values, 
                    success: true, 
                    positionError: posError, 
                    orientationError: oriError,
                    iterations: iter + 1 
                };
            }
        }

        const finalPose = computeEndPose(joints, values);
        return { 
            values, 
            success: false, 
            positionError: finalPose.position.distanceTo(targetPos),
            orientationError: finalPose.orientation.angleTo(targetOri),
            iterations: config.maxIterations 
        };
    }

    // ========================================
    // 雅可比矩阵迭代法
    // ========================================

    /**
     * 计算位置雅可比矩阵
     */
    function computePositionJacobian(joints, values) {
        const n = joints.length;
        const J = [];
        
        const fk = computeFullFK(joints, values);
        const endPos = fk.endPosition;
        
        for (let i = 0; i < n; i++) {
            const joint = joints[i];
            
            // 获取关节位置和轴向
            let T = new THREE.Matrix4();
            for (let j = 0; j < i; j++) {
                const T_base = buildBaseTransform(joints[j]);
                const T_joint = buildJointTransform(joints[j].type, values[j]);
                T = T.clone().multiply(T_base).multiply(T_joint);
            }
            if (i > 0) {
                T = T.clone().multiply(buildBaseTransform(joints[i]));
            }
            
            const jointPos = fk.positions[i];
            const zAxis = new THREE.Vector3(0, 0, 1);
            if (i > 0) {
                zAxis.applyMatrix4(T.clone().setPosition(0, 0, 0));
            }
            
            if (joint.type === 'R') {
                // 转动关节: J_i = z_i × (p_end - p_i)
                const toEnd = new THREE.Vector3().subVectors(endPos, jointPos);
                const col = new THREE.Vector3().crossVectors(zAxis, toEnd);
                J.push([col.x, col.y, col.z]);
            } else {
                // 平动关节: J_i = z_i
                J.push([zAxis.x, zAxis.y, zAxis.z]);
            }
        }
        
        return J;
    }

    /**
     * 使用阻尼最小二乘法求解IK
     */
    function solveJacobianDLS(joints, currentValues, targetPos, maxIter = 50) {
        if (joints.length === 0) {
            return { values: [], success: false, error: Infinity };
        }

        const values = [...currentValues];
        const lambda = config.dampingFactor;
        
        for (let iter = 0; iter < maxIter; iter++) {
            const currentPos = computeEndPosition(joints, values);
            const error = new THREE.Vector3().subVectors(targetPos, currentPos);
            const errorMag = error.length();
            
            if (errorMag < config.positionTolerance) {
                return { values, success: true, error: errorMag, iterations: iter + 1 };
            }
            
            // 计算雅可比矩阵
            const J = computePositionJacobian(joints, values);
            const n = joints.length;
            
            // J^T
            const JT = [];
            for (let i = 0; i < n; i++) {
                JT.push([J[i][0], J[i][1], J[i][2]]);
            }
            
            // J * J^T + λ²I (3x3 矩阵)
            const JJT = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
            for (let i = 0; i < 3; i++) {
                for (let j = 0; j < 3; j++) {
                    for (let k = 0; k < n; k++) {
                        JJT[i][j] += J[k][i] * J[k][j];
                    }
                }
                JJT[i][i] += lambda * lambda;
            }
            
            // 求逆 (3x3)
            const det = JJT[0][0] * (JJT[1][1] * JJT[2][2] - JJT[1][2] * JJT[2][1])
                      - JJT[0][1] * (JJT[1][0] * JJT[2][2] - JJT[1][2] * JJT[2][0])
                      + JJT[0][2] * (JJT[1][0] * JJT[2][1] - JJT[1][1] * JJT[2][0]);
            
            if (Math.abs(det) < 1e-10) continue;
            
            const invJJT = [
                [(JJT[1][1] * JJT[2][2] - JJT[1][2] * JJT[2][1]) / det,
                 (JJT[0][2] * JJT[2][1] - JJT[0][1] * JJT[2][2]) / det,
                 (JJT[0][1] * JJT[1][2] - JJT[0][2] * JJT[1][1]) / det],
                [(JJT[1][2] * JJT[2][0] - JJT[1][0] * JJT[2][2]) / det,
                 (JJT[0][0] * JJT[2][2] - JJT[0][2] * JJT[2][0]) / det,
                 (JJT[0][2] * JJT[1][0] - JJT[0][0] * JJT[1][2]) / det],
                [(JJT[1][0] * JJT[2][1] - JJT[1][1] * JJT[2][0]) / det,
                 (JJT[0][1] * JJT[2][0] - JJT[0][0] * JJT[2][1]) / det,
                 (JJT[0][0] * JJT[1][1] - JJT[0][1] * JJT[1][0]) / det]
            ];
            
            // Δq = J^T * (J * J^T + λ²I)^(-1) * e
            const temp = [
                invJJT[0][0] * error.x + invJJT[0][1] * error.y + invJJT[0][2] * error.z,
                invJJT[1][0] * error.x + invJJT[1][1] * error.y + invJJT[1][2] * error.z,
                invJJT[2][0] * error.x + invJJT[2][1] * error.y + invJJT[2][2] * error.z
            ];
            
            for (let i = 0; i < n; i++) {
                const dq = JT[i][0] * temp[0] + JT[i][1] * temp[1] + JT[i][2] * temp[2];
                values[i] += dq;
                
                if (joints[i].type === 'R') {
                    // 限制范围 - 使用关节配置的限位（转换为弧度）
                    const minLimit = joints[i].min !== undefined ? joints[i].min * Math.PI / 180 : -Math.PI;
                    const maxLimit = joints[i].max !== undefined ? joints[i].max * Math.PI / 180 : Math.PI;
                    values[i] = Math.max(minLimit, Math.min(maxLimit, values[i]));
                } else {
                    // 限制范围 - 使用关节配置的限位
                    const minLimit = joints[i].min !== undefined ? joints[i].min : -500;
                    const maxLimit = joints[i].max !== undefined ? joints[i].max : 500;
                    values[i] = Math.max(minLimit, Math.min(maxLimit, values[i]));
                }
            }
        }
        
        const finalError = computeEndPosition(joints, values).distanceTo(targetPos);
        return { values, success: false, error: finalError, iterations: maxIter };
    }

    // ========================================
    // 公开API
    // ========================================

    return {
        config,
        
        // FK
        computeFullFK,
        computeEndPosition,
        computeEndPose,
        
        // IK求解
        solveCCDPosition,
        solveCCDPose,
        solveJacobianDLS,
        
        // 默认求解器（使用CCD）
        solve: function(joints, currentValues, targetPos, targetOri = null) {
            if (targetOri) {
                return solveCCDPose(joints, currentValues, targetPos, targetOri);
            } else {
                return solveCCDPosition(joints, currentValues, targetPos);
            }
        }
    };

})();

// 导出为全局变量
if (typeof window !== 'undefined') {
    window.RobotArmIK = RobotArmIK;
}

