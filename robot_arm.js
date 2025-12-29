/**
 * 机械臂3D可视化工具
 * 基于物理结构描述的建模，支持MDH参数导出
 */

const RobotArm = (function() {
    'use strict';

    // ========================================
    // 状态变量
    // ========================================
    
    let scene, camera, renderer, controls, labelRenderer;
    let joints = [];          // 关节配置数据
    let jointValues = [];     // 当前关节变量值
    let jointMeshes = [];     // 关节网格
    let linkMeshes = [];      // 连杆网格
    let axisMeshes = [];      // 坐标轴网格
    let labelMeshes = [];     // 标签
    let workspacePoints = [];  // 工作空间点云数组（支持叠加多次渲染）
    let isRendering = false;
    let cancelRendering = false;
    let jointIdCounter = 0;
    
    // IK交互相关
    let endEffectorMesh = null;   // 末端执行器网格
    let transformControl = null;  // TransformControls
    let ikEnabled = true;
    let isIKDragging = false;
    const SCREENSHOT_SCALE = 3;   // 截图倍数（提高分辨率）

    // ========================================
    // 初始化
    // ========================================

    function init() {
        initScene();
        initIKInteraction();
        addDefaultJoint();
        animate();
        
        // 监听设置变化
        document.getElementById('link-radius').addEventListener('input', debounce(updateVisualization, 100));
        document.getElementById('axis-size').addEventListener('input', debounce(updateVisualization, 100));
        document.getElementById('joint-diameter').addEventListener('input', debounce(updateVisualization, 100));
        document.getElementById('joint-height').addEventListener('input', debounce(updateVisualization, 100));
        document.getElementById('show-axes').addEventListener('change', updateVisualization);
        document.getElementById('show-labels').addEventListener('change', updateVisualization);
        
        // 初始化侧边栏状态
        updateSidebarToggleIcon();
    }
    
    function toggleSidebar() {
        const app = document.getElementById('app');
        const panel = document.getElementById('input-panel');
        const toggleIcon = document.getElementById('toggle-icon');
        
        const isCollapsed = panel.classList.contains('collapsed');
        
        app.classList.toggle('sidebar-collapsed');
        panel.classList.toggle('collapsed');
        
        // 如果展开，强制设置宽度
        if (isCollapsed) {
            // 展开时，确保宽度正确恢复
            setTimeout(() => {
                panel.style.width = '50%';
                panel.style.maxWidth = '50%';
                panel.style.minWidth = '400px';
            }, 10);
        } else {
            // 折叠时，清除内联样式，让CSS控制
            panel.style.width = '';
            panel.style.maxWidth = '';
            panel.style.minWidth = '';
        }
        
        updateSidebarToggleIcon();
        
        // 窗口大小改变时更新canvas
        setTimeout(() => {
            onWindowResize();
        }, 300);
    }
    
    function updateSidebarToggleIcon() {
        const panel = document.getElementById('input-panel');
        const toggleIcon = document.getElementById('toggle-icon');
        
        if (panel.classList.contains('collapsed')) {
            toggleIcon.textContent = '▶';
        } else {
            toggleIcon.textContent = '◀';
        }
    }

    /**
     * 使用体素计数近似工作空间体积
     * @param {THREE.Vector3[]} pts 点云
     * @param {number} voxelSize 体素边长
     * @returns {number} 体积 (mm^3)
     */
    function computeWorkspaceVolume(pts, voxelSize) {
        if (!pts || pts.length === 0 || voxelSize <= 0) return 0;
        const occupied = new Set();
        const inv = 1 / voxelSize;
        for (let i = 0; i < pts.length; i++) {
            const p = pts[i];
            const ix = Math.floor(p.x * inv);
            const iy = Math.floor(p.y * inv);
            const iz = Math.floor(p.z * inv);
            occupied.add(`${ix},${iy},${iz}`);
        }
        const volumeMm3 = occupied.size * Math.pow(voxelSize, 3);
        return volumeMm3 / 1e9; // 转换为立方米
    }
    
    // ========================================
    // 灵活空间分析 - 基于可操作度和各向同性指数
    // ========================================
    
    /**
     * 已删除：基于采样的方法和Worker相关代码
     * 现在使用快速的几何指标方法
     */
    
    /**
     * 【已废弃】创建内联Worker
     * 保留代码结构以防未来需要
     */
    function createInlineWorker_DEPRECATED() {
        const workerCode = `
        // Worker代码（内联）
        'use strict';
        
        class Vec3 {
            constructor(x = 0, y = 0, z = 0) {
                this.x = x; this.y = y; this.z = z;
            }
            clone() { return new Vec3(this.x, this.y, this.z); }
            add(v) { this.x += v.x; this.y += v.y; this.z += v.z; return this; }
            sub(v) { this.x -= v.x; this.y -= v.y; this.z -= v.z; return this; }
            multiplyScalar(s) { this.x *= s; this.y *= s; this.z *= s; return this; }
            dot(v) { return this.x * v.x + this.y * v.y + this.z * v.z; }
            cross(v) {
                const x = this.y * v.z - this.z * v.y;
                const y = this.z * v.x - this.x * v.z;
                const z = this.x * v.y - this.y * v.x;
                return new Vec3(x, y, z);
            }
            length() { return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z); }
            normalize() {
                const len = this.length();
                if (len > 0) this.multiplyScalar(1 / len);
                return this;
            }
            distanceTo(v) {
                const dx = this.x - v.x, dy = this.y - v.y, dz = this.z - v.z;
                return Math.sqrt(dx * dx + dy * dy + dz * dz);
            }
        }
        
        class Mat4 {
            constructor() {
                this.elements = new Float32Array(16);
                this.identity();
            }
            identity() {
                const e = this.elements;
                e[0]=1;e[4]=0;e[8]=0;e[12]=0;e[1]=0;e[5]=1;e[9]=0;e[13]=0;
                e[2]=0;e[6]=0;e[10]=1;e[14]=0;e[3]=0;e[7]=0;e[11]=0;e[15]=1;
                return this;
            }
            clone() {
                const m = new Mat4();
                m.elements.set(this.elements);
                return m;
            }
            multiply(m) {
                const ae = this.elements, be = m.elements;
                const result = new Float32Array(16);
                for (let i = 0; i < 4; i++) {
                    for (let j = 0; j < 4; j++) {
                        result[i*4+j] = ae[i*4]*be[j] + ae[i*4+1]*be[4+j] + 
                                        ae[i*4+2]*be[8+j] + ae[i*4+3]*be[12+j];
                    }
                }
                this.elements = result;
                return this;
            }
            makeTranslation(x,y,z) {
                this.identity(); this.elements[12]=x; this.elements[13]=y; this.elements[14]=z;
                return this;
            }
            makeRotationZ(theta) {
                const c=Math.cos(theta), s=Math.sin(theta);
                this.identity(); this.elements[0]=c; this.elements[1]=s;
                this.elements[4]=-s; this.elements[5]=c;
                return this;
            }
            makeRotationFromEuler(rx,ry,rz) {
                const cx=Math.cos(rx),sx=Math.sin(rx),cy=Math.cos(ry),sy=Math.sin(ry);
                const cz=Math.cos(rz),sz=Math.sin(rz);
                const e=this.elements;
                e[0]=cy*cz;e[1]=cy*sz;e[2]=-sy;e[3]=0;
                e[4]=sx*sy*cz-cx*sz;e[5]=sx*sy*sz+cx*cz;e[6]=sx*cy;e[7]=0;
                e[8]=cx*sy*cz+sx*sz;e[9]=cx*sy*sz-sx*cz;e[10]=cx*cy;e[11]=0;
                e[12]=0;e[13]=0;e[14]=0;e[15]=1;
                return this;
            }
            getPosition() { return new Vec3(this.elements[12],this.elements[13],this.elements[14]); }
            transformVector(v) {
                const e=this.elements, x=v.x, y=v.y, z=v.z;
                return new Vec3(e[0]*x+e[4]*y+e[8]*z, e[1]*x+e[5]*y+e[9]*z, e[2]*x+e[6]*y+e[10]*z);
            }
        }
        
        function buildBaseTransform(joint) {
            const trans = new Mat4().makeTranslation(joint.x, joint.y, joint.z);
            const rot = new Mat4().makeRotationFromEuler(
                joint.rx*Math.PI/180, joint.ry*Math.PI/180, joint.rz*Math.PI/180);
            return trans.clone().multiply(rot);
        }
        
        function buildJointTransform(type,value) {
            return type==='R' ? new Mat4().makeRotationZ(value) : new Mat4().makeTranslation(0,0,value);
        }
        
        function computeEndPosition(joints,values) {
            let T=new Mat4();
            for(let i=0;i<joints.length;i++){
                T=T.clone().multiply(buildBaseTransform(joints[i])).multiply(buildJointTransform(joints[i].type,values[i]));
            }
            return T.getPosition();
        }
        
        function computeFullFK(joints,values) {
            const positions=[new Vec3(0,0,0)];
            let T=new Mat4();
            for(let i=0;i<joints.length;i++){
                T=T.clone().multiply(buildBaseTransform(joints[i])).multiply(buildJointTransform(joints[i].type,values[i]));
                positions.push(T.getPosition());
            }
            return {positions,endTransform:T};
        }
        
        function solveCCDPosition(joints,currentValues,targetPos) {
            if(joints.length===0) return null;
            const values=[...currentValues];
            const maxIter=50, tolerance=15;
            
            for(let iter=0;iter<maxIter;iter++){
                for(let i=joints.length-1;i>=0;i--){
                    const joint=joints[i];
                    if(joint.type==='R'){
                        const fk=computeFullFK(joints,values);
                        const jointPos=fk.positions[i], endPos=computeEndPosition(joints,values);
                        let T=new Mat4();
                        for(let j=0;j<i;j++){
                            T=T.clone().multiply(buildBaseTransform(joints[j])).multiply(buildJointTransform(joints[j].type,values[j]));
                        }
                        if(i>0) T=T.clone().multiply(buildBaseTransform(joints[i]));
                        const zAxis=T.transformVector(new Vec3(0,0,1)).normalize();
                        const toEnd=endPos.clone().sub(jointPos);
                        const toTarget=targetPos.clone().sub(jointPos);
                        const toEndProj=toEnd.clone().sub(zAxis.clone().multiplyScalar(toEnd.dot(zAxis)));
                        const toTargetProj=toTarget.clone().sub(zAxis.clone().multiplyScalar(toTarget.dot(zAxis)));
                        if(toEndProj.length()<0.001||toTargetProj.length()<0.001) continue;
                        toEndProj.normalize(); toTargetProj.normalize();
                        let angle=Math.acos(Math.max(-1,Math.min(1,toEndProj.dot(toTargetProj))));
                        const cross=toEndProj.cross(toTargetProj);
                        if(cross.dot(zAxis)<0) angle=-angle;
                        angle=Math.max(-0.5,Math.min(0.5,angle));
                        values[i]+=angle;
                        const minLimit=joint.min!==undefined?joint.min*Math.PI/180:-Math.PI;
                        const maxLimit=joint.max!==undefined?joint.max*Math.PI/180:Math.PI;
                        values[i]=Math.max(minLimit,Math.min(maxLimit,values[i]));
                    }
                }
                const currentEnd=computeEndPosition(joints,values);
                const error=currentEnd.distanceTo(targetPos);
                if(error<tolerance) return {values,success:true,error};
            }
            const finalError=computeEndPosition(joints,values).distanceTo(targetPos);
            return {values,success:finalError<tolerance,error:finalError};
        }
        
        self.onmessage = function(e) {
            const {type,data} = e.data;
            if(type==='test_points'){
                const {points,joints,orientations,threshold,workerId} = data;
                const results=[];
                for(let i=0;i<points.length;i++){
                    const point=new Vec3(points[i].x,points[i].y,points[i].z);
                    let reachableCount=0;
                    for(let j=0;j<orientations.length;j++){
                        const initialGuess=joints.map(()=>0);
                        const result=solveCCDPosition(joints,initialGuess,point);
                        if(result && result.success) reachableCount++;
                    }
                    const coverage=reachableCount/orientations.length;
                    results.push({point:points[i],isDexterous:coverage>=threshold,coverage});
                    if(i%5===0){
                        self.postMessage({type:'progress',workerId,processed:i+1,total:points.length});
                    }
                }
                self.postMessage({type:'complete',workerId,results});
            }
        };
        `;
        
        const blob = new Blob([workerCode], { type: 'application/javascript' });
        return new Worker(URL.createObjectURL(blob));
    }
    
    /**
     * 使用Web Workers并行测试灵活空间
     */
    async function testDexterousWorkspaceParallel(points, orientations, threshold, numWorkers, onProgress) {
        return new Promise((resolve, reject) => {
            const workers = [];
            const results = [];
            let completedWorkers = 0;
            const workerProgress = new Array(numWorkers).fill(0);
            
            // 将点分配给各个Worker
            const pointsPerWorker = Math.ceil(points.length / numWorkers);
            
            for (let i = 0; i < numWorkers; i++) {
                const worker = createInlineWorker();
                const startIdx = i * pointsPerWorker;
                const endIdx = Math.min((i + 1) * pointsPerWorker, points.length);
                const workerPoints = points.slice(startIdx, endIdx);
                
                if (workerPoints.length === 0) continue;
                
                worker.onmessage = function(e) {
                    const { type, workerId, processed, total, results: workerResults } = e.data;
                    
                    if (type === 'progress') {
                        workerProgress[i] = processed / total;
                        const totalProgress = workerProgress.reduce((a, b) => a + b, 0) / numWorkers;
                        if (onProgress) onProgress(totalProgress);
                    } else if (type === 'complete') {
                        results.push(...workerResults);
                        completedWorkers++;
                        
                        worker.terminate();
                        
                        if (completedWorkers === workers.length) {
                            resolve(results);
                        }
                    }
                };
                
                worker.onerror = function(error) {
                    console.error('Worker error:', error);
                    worker.terminate();
                    reject(error);
                };
                
                // 发送任务
                worker.postMessage({
                    type: 'test_points',
                    data: {
                        points: workerPoints.map(p => ({ x: p.x, y: p.y, z: p.z })),
                        joints: joints,
                        orientations: orientations.map(q => ({
                            x: q.x, y: q.y, z: q.z, w: q.w
                        })),
                        threshold: threshold,
                        workerId: i
                    }
                });
                
                workers.push(worker);
            }
            
            if (workers.length === 0) {
                resolve([]);
            }
        });
    }
    
    // ========================================
    // 灵活空间分析 - 基于可操作度和条件数
    // ========================================
    
    // 灵活度计算常量配置
    const DEXTERITY_CONFIG = {
        // 特征长度（消除量纲不一致性，Angeles 1992）
        CHARACTERISTIC_LENGTH: 700,       // mm（对应典型机械臂臂展0.7m）
        
        // 条件数评价区间（基于工业机器人实践 & 文献）
        KAPPA_EXCELLENT: 5,               // 优秀：κ < 5（接近各向同性）
        KAPPA_GOOD: 20,                   // 良好：κ < 20
        KAPPA_ACCEPTABLE: 80,             // 可接受：κ < 80
        KAPPA_LIMIT: 100,                 // 极限：κ < 100（Angeles 1997建议）
        
        // 综合灵活度权重（Klein & Blaho 1987）
        WEIGHT_MANIPULABILITY: 0.55,      // 可操作度权重（略倾向运动能力）
        WEIGHT_ISOTROPY: 0.45,            // 各向同性权重（均衡性）
        
        // 数值稳定性阈值
        MIN_DET_THRESHOLD: 1e-10,         // 行列式最小阈值（防止除零）
        MIN_TRACE_THRESHOLD: 1e-10,       // 迹最小阈值（防止除零）
        
        // IK求解容差
        MAX_IK_ERROR: 20,                 // IK位置误差上限（mm）
        
        // 软约束参数（平滑过渡）
        SIGMOID_STEEPNESS: 0.05           // Sigmoid陡度（文献建议0.05-0.1）
    };
    
    /**
     * 计算尺度一致的雅可比矩阵（Scale-Consistent Jacobian）
     * 
     * 使用特征长度 l_c 消除平移与旋转的量纲不一致性
     * J' = [J_v / l_c; J_ω]
     * 
     * 参考文献：
     * - Angeles, J. (1992). "The Design of Isotropic Manipulator Architectures..."
     * - Cardou & Bouchard (2010). "Kinematic-Sensitivity Indices..."
     * 
     * @param {Array} config 关节配置（弧度/mm）
     * @returns {Array} 6×n 尺度一致雅可比矩阵（无量纲）
     */
    function computeJacobian(config) {
        const n = joints.length;
        const J = [];
        const l_c = DEXTERITY_CONFIG.CHARACTERISTIC_LENGTH; // 特征长度（mm）
        
        // 计算FK
        const fk = computeFK(config);
        const endPos = fk.positions[fk.positions.length - 1];
        
        for (let i = 0; i < n; i++) {
            const joint = joints[i];
            const jointPos = fk.positions[i];
            const zAxis = fk.zAxes[i + 1]; // 关节轴方向
            
            if (joint.type === 'R') {
                // 转动关节：J_v = z × (p_end - p_joint), J_ω = z
                const toEnd = new THREE.Vector3().subVectors(endPos, jointPos);
                const Jv = new THREE.Vector3().crossVectors(zAxis, toEnd);
                const Jw = zAxis.clone();
                
                // 尺度一致化：平移部分除以特征长度
                J.push([Jv.x/l_c, Jv.y/l_c, Jv.z/l_c, Jw.x, Jw.y, Jw.z]);
            } else {
                // 平动关节：J_v = z, J_ω = 0
                // 同样需要归一化平移部分
                J.push([zAxis.x/l_c, zAxis.y/l_c, zAxis.z/l_c, 0, 0, 0]);
            }
        }
        
        return J; // n × 6 矩阵（尺度一致，无量纲）
    }
    
    /**
     * 计算归一化雅可比矩阵的 J'·J'^T（用于可操作度和条件数计算）
     * 
     * 注意：J' 已经过尺度一致化处理（J_v/l_c），因此：
     * - det(J'J'^T) 是无量纲量，具有明确物理意义
     * - κ(J') 与机器人尺寸无关，可比较不同机器人
     * 
     * @param {Array} config 关节配置
     * @returns {Array|null} 3×3矩阵 J'J'^T，如果失败返回null
     */
    function computeJJT(config) {
        const J = computeJacobian(config);  // 获取尺度一致雅可比 J'
        if (J.length === 0) return null;
        
        const n = J.length;
        // 提取平移部分（前3列，已归一化）
        const Jv = J.map(row => [row[0], row[1], row[2]]);
        
        // 计算 J'_v · J'^T_v (3×3矩阵)
        const JJT = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ];
        
        for (let i = 0; i < 3; i++) {
            for (let j = 0; j < 3; j++) {
                for (let k = 0; k < n; k++) {
                    JJT[i][j] += Jv[k][i] * Jv[k][j];
                }
            }
        }
        
        return JJT;  // 无量纲3×3矩阵
    }
    
    /**
     * 计算尺度一致可操作度 M' (Yoshikawa 1985)
     * M' = sqrt(det(J'·J'^T))
     * 
     * 物理意义：
     * - 刻画机器人在任务空间中的"瞬时运动能力"
     * - 无量纲量，可比较不同尺寸机器人
     * - 值越大表示运动能力越强
     * 
     * @param {Array} JJT 尺度一致的 J'·J'^T (3×3无量纲矩阵)
     * @returns {number} 可操作度指标（无量纲）
     */
    function computeManipulability(JJT) {
        if (!JJT) return 0;
        
        // 计算3×3矩阵的行列式
        const det = JJT[0][0] * (JJT[1][1] * JJT[2][2] - JJT[1][2] * JJT[2][1])
                  - JJT[0][1] * (JJT[1][0] * JJT[2][2] - JJT[1][2] * JJT[2][0])
                  + JJT[0][2] * (JJT[1][0] * JJT[2][1] - JJT[1][1] * JJT[2][0]);
        
        return Math.sqrt(Math.max(0, det));
    }
    
    /**
     * 计算条件数 κ(J') = sqrt(λ_max / λ_min)
     * 
     * 物理意义：
     * - 刻画机器人运动能力的"各向同性程度"
     * - κ=1: 完美各向同性（最优）
     * - κ越大: 某方向运动能力越弱（接近奇异）
     * - 由于J'无量纲，κ与机器人尺寸无关
     * 
     * @param {Array} JJT 尺度一致的 J'·J'^T (3×3无量纲矩阵)
     * @returns {number} 条件数（≥1，越小越好）
     */
    function computeConditionNumber(JJT) {
        if (!JJT) return Infinity;
        
        // 计算迹和行列式（数值稳定性检查）
        const trace = JJT[0][0] + JJT[1][1] + JJT[2][2];
        const det = JJT[0][0] * (JJT[1][1] * JJT[2][2] - JJT[1][2] * JJT[2][1])
                  - JJT[0][1] * (JJT[1][0] * JJT[2][2] - JJT[1][2] * JJT[2][0])
                  + JJT[0][2] * (JJT[1][0] * JJT[2][1] - JJT[1][1] * JJT[2][0]);
        
        // 数值稳定性检查：避免除零
        if (det < DEXTERITY_CONFIG.MIN_DET_THRESHOLD || 
            trace < DEXTERITY_CONFIG.MIN_TRACE_THRESHOLD) {
            return Infinity;
        }
        
        // 特征值近似：λ_max ≈ trace, λ_min ≈ det/trace²
        const lambda_max = trace;
        const lambda_min = det / (trace * trace);
        
        if (lambda_min < DEXTERITY_CONFIG.MIN_DET_THRESHOLD) {
            return Infinity;
        }
        
        // 条件数 κ = sqrt(λ_max / λ_min)
        return Math.sqrt(lambda_max / lambda_min);
    }
    
    /**
     * 计算归一化各向同性指数（对数尺度 + Sigmoid软约束）
     * 
     * 方法：
     * 1. 对数映射：log_score = (log(κ_good) - log(κ)) / (log(κ_good) - log(κ_excellent))
     * 2. Sigmoid软约束：处理超出区间的情况，平滑过渡
     * 
     * @param {number} kappa 条件数
     * @returns {number} 归一化各向同性指数（0-1，越大越好）
     */
    function computeNormalizedIsotropy(kappa) {
        const cfg = DEXTERITY_CONFIG;
        
        // 数值稳定性检查
        if (!isFinite(kappa) || kappa < 1) {
            return kappa < 1 ? 1.0 : 0.0;
        }
        
        // 极限情况：κ > 150 时使用Sigmoid软衰减
        if (kappa > cfg.KAPPA_LIMIT) {
            // 软约束：Sigmoid衰减，避免硬截断
            // I_norm = exp(-steepness * (κ - κ_limit))
            const excess = kappa - cfg.KAPPA_LIMIT;
            const decay = Math.exp(-cfg.SIGMOID_STEEPNESS * excess);
            return Math.max(0, decay * 0.1);  // 最多保留10%得分
        }
        
        // 对数尺度归一化（更符合条件数的指数特性）
        // 将 [κ_excellent, κ_good] 映射到 [1.0, 0.5]
        // 将 [κ_good, κ_acceptable] 映射到 [0.5, 0.1]
        // 将 [κ_acceptable, κ_limit] 映射到 [0.1, 0.0]
        
        let I_norm;
        
        if (kappa <= cfg.KAPPA_EXCELLENT) {
            // 优秀区间：κ ≤ 5，接近完美
            I_norm = 1.0;
            
        } else if (kappa <= cfg.KAPPA_GOOD) {
            // 优秀到良好：κ ∈ (5, 20]，对数插值
            // log scale: 避免线性插值的不均匀分布
            const logKappa = Math.log(kappa);
            const logMin = Math.log(cfg.KAPPA_EXCELLENT);
            const logMax = Math.log(cfg.KAPPA_GOOD);
            const ratio = (logKappa - logMin) / (logMax - logMin);
            I_norm = 1.0 - 0.5 * ratio;  // 1.0 → 0.5
            
        } else if (kappa <= cfg.KAPPA_ACCEPTABLE) {
            // 良好到可接受：κ ∈ (20, 80]
            const logKappa = Math.log(kappa);
            const logMin = Math.log(cfg.KAPPA_GOOD);
            const logMax = Math.log(cfg.KAPPA_ACCEPTABLE);
            const ratio = (logKappa - logMin) / (logMax - logMin);
            I_norm = 0.5 - 0.4 * ratio;  // 0.5 → 0.1
            
        } else {
            // 可接受到极限：κ ∈ (80, 150]
            const logKappa = Math.log(kappa);
            const logMin = Math.log(cfg.KAPPA_ACCEPTABLE);
            const logMax = Math.log(cfg.KAPPA_LIMIT);
            const ratio = (logKappa - logMin) / (logMax - logMin);
            I_norm = 0.1 - 0.1 * ratio;  // 0.1 → 0.0
        }
        
        // 确保在 [0, 1] 范围内
        return Math.max(0, Math.min(1, I_norm));
    }
    
    /**
     * 计算综合灵活度指数
     * D = 0.4 × M_norm + 0.6 × I_norm （满足硬约束前提下）
     * @param {THREE.Vector3} position 测试点位置
     * @param {number} maxManipulability 工作空间内的最大可操作度（用于归一化）
     * @returns {Object} {dexterity, manipulability, isotropy, rawManip, kappa}
     */
    function computeDexterityIndex(position, maxManipulability = 1.0) {
        // 初始化返回值（不可达情况）
        const nullResult = { 
            dexterity: 0, 
            manipulability: 0, 
            isotropy: 0, 
            rawManip: 0, 
            kappa: Infinity 
        };
        
        if (joints.length === 0) {
            return nullResult;
        }
        
        // 步骤1: 使用IK求解找到可达该点的关节配置
        const targetPos = position.clone();
        const result = RobotArmIK ? RobotArmIK.solveCCDPosition(joints, jointValues, targetPos) : null;
        
        // IK失败或误差过大，判定为不可达
        if (!result || !result.values || result.error > DEXTERITY_CONFIG.MAX_IK_ERROR) {
            return nullResult;
        }
        
        // 步骤2: 计算雅可比矩阵的 J·J^T
        const JJT = computeJJT(result.values);
        if (!JJT) {
            return nullResult;
        }
        
        // 步骤3: 计算原始可操作度 M
        const rawManip = computeManipulability(JJT);
        
        // 步骤4: 计算归一化可操作度 M_norm = M / M_max
        const M_norm = maxManipulability > DEXTERITY_CONFIG.MIN_DET_THRESHOLD 
            ? Math.min(1, rawManip / maxManipulability) 
            : 0;
        
        // 步骤5: 计算条件数 κ
        const kappa = computeConditionNumber(JJT);
        
        // 步骤6: 计算归一化各向同性指数 I_norm（包含软约束）
        const I_norm = computeNormalizedIsotropy(kappa);
        
        // 步骤7: 计算综合灵活度指数（使用均衡权重）
        // D = 0.5 × M_norm + 0.5 × I_norm
        const dexterity = DEXTERITY_CONFIG.WEIGHT_MANIPULABILITY * M_norm + 
                         DEXTERITY_CONFIG.WEIGHT_ISOTROPY * I_norm;
        
        return {
            dexterity: dexterity,      // 综合灵活度 [0, 1]
            manipulability: M_norm,    // 归一化可操作度 [0, 1]
            isotropy: I_norm,          // 归一化各向同性 [0, 1]
            rawManip: rawManip,        // 原始可操作度（用于第一阶段统计）
            kappa: kappa               // 条件数（用于调试）
        };
    }

    // ========================================
    // IK交互初始化
    // ========================================

    function initIKInteraction() {
        createEndEffector();
        createTransformControl();
    }

    function createEndEffector() {
        // 末端执行器指示器（坐标轴形式的小工具）
        const size = 30;
        
        // 创建一个小的坐标轴指示器
        endEffectorMesh = new THREE.Group();
        
        // 中心球体
        const sphereGeom = new THREE.SphereGeometry(8, 16, 16);
        const sphereMat = new THREE.MeshPhongMaterial({
            color: 0xffff00,
            emissive: 0x444400,
            transparent: true,
            opacity: 0.9
        });
        const sphere = new THREE.Mesh(sphereGeom, sphereMat);
        endEffectorMesh.add(sphere);
        
        // X轴（红）
        const xArrow = new THREE.ArrowHelper(
            new THREE.Vector3(1, 0, 0), 
            new THREE.Vector3(0, 0, 0), 
            size, 0xff0000, size * 0.3, size * 0.15
        );
        endEffectorMesh.add(xArrow);
        
        // Y轴（绿）
        const yArrow = new THREE.ArrowHelper(
            new THREE.Vector3(0, 1, 0), 
            new THREE.Vector3(0, 0, 0), 
            size, 0x00ff00, size * 0.3, size * 0.15
        );
        endEffectorMesh.add(yArrow);
        
        // Z轴（蓝）
        const zArrow = new THREE.ArrowHelper(
            new THREE.Vector3(0, 0, 1), 
            new THREE.Vector3(0, 0, 0), 
            size, 0x0000ff, size * 0.3, size * 0.15
        );
        endEffectorMesh.add(zArrow);
        
        endEffectorMesh.visible = false;
        scene.add(endEffectorMesh);
    }

    function createTransformControl() {
        transformControl = new THREE.TransformControls(camera, renderer.domElement);
        transformControl.setSize(0.8);
        transformControl.setSpace('local');
        scene.add(transformControl);
        
        // 当开始拖拽时禁用OrbitControls
        transformControl.addEventListener('dragging-changed', function(event) {
            controls.enabled = !event.value;
            isIKDragging = event.value;
        });
        
        // 当拖拽时实时更新IK
        transformControl.addEventListener('change', function() {
            if (isIKDragging && endEffectorMesh && joints.length > 0) {
                solveIK();
            }
        });
        
        // 键盘快捷键切换模式
        window.addEventListener('keydown', function(event) {
            if (!ikEnabled) return;
            
            switch (event.key.toLowerCase()) {
                case 'g': // Grab - 平移
                    transformControl.setMode('translate');
                    break;
                case 'r': // Rotate - 旋转
                    transformControl.setMode('rotate');
                    break;
                case 'x': // X轴约束
                    if (transformControl.showX) {
                        transformControl.showX = true;
                        transformControl.showY = false;
                        transformControl.showZ = false;
                    } else {
                        transformControl.showX = true;
                        transformControl.showY = true;
                        transformControl.showZ = true;
                    }
                    break;
                case 'y': // Y轴约束
                    transformControl.showX = false;
                    transformControl.showY = true;
                    transformControl.showZ = false;
                    break;
                case 'z': // Z轴约束
                    transformControl.showX = false;
                    transformControl.showY = false;
                    transformControl.showZ = true;
                    break;
                case 'escape':
                    transformControl.showX = true;
                    transformControl.showY = true;
                    transformControl.showZ = true;
                    break;
            }
        });
    }

    function updateEndEffectorPosition() {
        if (!endEffectorMesh || joints.length === 0) {
            if (endEffectorMesh) endEffectorMesh.visible = false;
            if (transformControl) transformControl.detach();
            return;
        }
        
        const pose = computeEndPose(jointValues);
        
        // 只在非拖拽时更新位置，避免循环
        if (!isIKDragging) {
            endEffectorMesh.position.copy(pose.position);
            endEffectorMesh.quaternion.copy(pose.orientation);
        }
        
        endEffectorMesh.visible = ikEnabled;
        
        // 附加TransformControls
        if (ikEnabled && !transformControl.object) {
            transformControl.attach(endEffectorMesh);
        }
    }

    function computeEndPose(values) {
        if (typeof RobotArmIK !== 'undefined') {
            return RobotArmIK.computeEndPose(joints, values);
        }
        // 备用计算
        let T = new THREE.Matrix4();
        for (let i = 0; i < joints.length; i++) {
            const T_base = buildBaseTransform(joints[i]);
            const T_joint = buildJointTransform(joints[i], values[i]);
            T = T.clone().multiply(T_base).multiply(T_joint);
        }
        const pos = new THREE.Vector3().setFromMatrixPosition(T);
        const quat = new THREE.Quaternion().setFromRotationMatrix(T);
        return { position: pos, orientation: quat };
    }

    // ========================================
    // IK求解
    // ========================================

    function solveIK() {
        if (typeof RobotArmIK === 'undefined' || joints.length === 0) return;
        if (!endEffectorMesh) return;
        
        const targetPos = endEffectorMesh.position.clone();
        const targetOri = endEffectorMesh.quaternion.clone();
        
        // 根据当前模式决定求解方式
        const mode = transformControl.mode;
        let result;
        
        if (mode === 'rotate') {
            // 旋转模式：同时求解位置和姿态
            result = RobotArmIK.solveCCDPose(joints, jointValues, targetPos, targetOri, 0.5);
        } else {
            // 平移模式：只求解位置
            result = RobotArmIK.solveCCDPosition(joints, jointValues, targetPos);
        }
        
        if (result.values) {
            jointValues = result.values;
            
            // 更新可视化（但不更新末端执行器位置，避免抖动）
            updateVisualizationWithoutEndEffector();
            updateControlPanel();
        }
    }

    function updateVisualizationWithoutEndEffector() {
        if (!scene) return;

        clearVisualization();

        if (joints.length === 0) {
            updateStatus();
            return;
        }

        const radius = parseFloat(document.getElementById('link-radius').value) || 25;
        const axisSize = parseFloat(document.getElementById('axis-size').value) || 3;
        const jointDiameter = parseFloat(document.getElementById('joint-diameter').value) || 60;
        const jointHeight = parseFloat(document.getElementById('joint-height').value) || 60;
        const showAxes = document.getElementById('show-axes').checked;
        const showLabels = document.getElementById('show-labels').checked;

        // 计算FK
        const fk = computeFK(jointValues);
        const { transforms, positions, xAxes, yAxes, zAxes } = fk;

        // 材质
        const jointMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xef4444,
            shininess: 80
        });
        const linkMaterial = new THREE.MeshPhongMaterial({ 
            color: 0x38bdf8,
            shininess: 60,
            transparent: true,
            opacity: 0.85
        });
        const baseMaterial = new THREE.MeshPhongMaterial({ 
            color: 0x10b981,
            shininess: 80
        });

        // 绘制基座
        const baseGeom = new THREE.CylinderGeometry(radius * 2.5, radius * 3, radius * 2, 32);
        const baseMesh = new THREE.Mesh(baseGeom, baseMaterial);
        baseMesh.rotation.x = Math.PI / 2;
        baseMesh.position.z = -radius;
        scene.add(baseMesh);
        jointMeshes.push(baseMesh);

        // 绘制关节和连杆
        const jointRadius = jointDiameter / 2;

        // 基座标签
        if (showLabels && positions.length > 0) {
            const basePos = positions[0];
            const label = createLabel('基座{0}', basePos.clone().add(new THREE.Vector3(0, 0, radius * 3)), '#10b981');
            scene.add(label);
            labelMeshes.push(label);
        }

        positions.forEach((pos, i) => {
            if (i === 0) {
                // 基座保持原样（已经是圆柱体）
                return;
            }
            
            // 检查关节是否可见（positions[i]对应joints[i-1]）
            const joint = joints[i - 1];
            const isVisible = joint.visible !== false;
            
            if (isVisible) {
                // 创建圆柱体关节
                const jointGeom = new THREE.CylinderGeometry(jointRadius, jointRadius, jointHeight, 32);
                const jointMesh = new THREE.Mesh(jointGeom, jointMaterial);
                
                // 获取关节Z轴方向（关节轴向）
                const zAxis = zAxes[i].clone().normalize();
                
                // 将圆柱体沿Z轴方向放置
                // Three.js的CylinderGeometry默认沿Y轴，需要旋转到Z轴
                const up = new THREE.Vector3(0, 1, 0);
                const quat = new THREE.Quaternion();
                quat.setFromUnitVectors(up, zAxis);
                jointMesh.setRotationFromQuaternion(quat);
                
                // 设置位置
                jointMesh.position.copy(pos);
                scene.add(jointMesh);
                jointMeshes.push(jointMesh);

                if (showLabels) {
                    const labelText = `{${i}}`;
                    const labelColor = '#f1f5f9';
                    // 标签位置：在圆柱体上方，沿Z轴方向偏移
                    const labelOffset = zAxis.clone().multiplyScalar(jointHeight / 2 + jointRadius + 5);
                    const label = createLabel(labelText, pos.clone().add(labelOffset), labelColor);
                    scene.add(label);
                    labelMeshes.push(label);
                }
            }

            // 坐标系显示（需要同时满足全局显示和关节可见）
            if (showAxes && isVisible && i > 0) {
                const transform = transforms[i - 1];
                const axesHelper = new THREE.AxesHelper(radius * 6 * axisSize);
                axesHelper.position.copy(pos);
                
                const rotation = new THREE.Matrix4();
                rotation.extractRotation(transform.matrix);
                axesHelper.setRotationFromMatrix(rotation);
                
                scene.add(axesHelper);
                axisMeshes.push(axesHelper);

                const axisColor = transform.type === 'R' ? 0xff00ff : 0xffa500;
                const arrow = new THREE.ArrowHelper(
                    zAxes[i].clone().normalize(),
                    pos,
                    radius * 8 * axisSize,
                    axisColor,
                    radius * 2 * axisSize,
                    radius * axisSize
                );
                scene.add(arrow);
                axisMeshes.push(arrow);
            }

            if (showAxes && i === 0) {
                const axesHelper = new THREE.AxesHelper(radius * 8 * axisSize);
                axesHelper.position.copy(pos);
                scene.add(axesHelper);
                axisMeshes.push(axesHelper);
            }
        });

        // 绘制连杆
        // 从关节1到关节2的连杆开始（i=2时，连接positions[1]和positions[2]）
        for (let i = 2; i < positions.length; i++) {
            const startPos = positions[i - 1];
            const endPos = positions[i];
            const linkIndex = i - 1;
            
            // 获取关节坐标系信息
            // positions[i-1]对应transforms[i-2]（如果i>=2）
            // 对于i=2: positions[1]是关节1，对应transforms[0]
            const startTransform = transforms[i - 2];
            const endTransform = transforms[i - 1];
            
            const xAxisStart = startTransform.xAxis.clone().normalize();
            const yAxisStart = startTransform.yAxis.clone().normalize();
            const zAxisStart = startTransform.zAxis.clone().normalize();
            const zAxisEnd = endTransform.zAxis.clone().normalize();
            
            // 获取该关节的连杆模式
            const startJoint = joints[i - 2];
            const linkMode = startJoint.linkMode || 'auto';
            
            // 计算连杆路径
            const path = computeLinkPath(startPos, xAxisStart, yAxisStart, zAxisStart, endPos, zAxisEnd, linkMode);
            
            // 绘制连杆段
            for (let j = 0; j < path.points.length - 1; j++) {
                const segStart = path.points[j];
                const segEnd = path.points[j + 1];
                const mesh = createLinkCylinder(segStart, segEnd, radius, linkMaterial);
                
                if (mesh) {
                    scene.add(mesh);
                    linkMeshes.push(mesh);
                }
            }
            
            // 在转折点添加连接球体
            if (path.points.length > 2) {
                for (let j = 1; j < path.points.length - 1; j++) {
                    const sphereGeom = new THREE.SphereGeometry(radius, 16, 16);
                    const sphereMesh = new THREE.Mesh(sphereGeom, linkMaterial);
                    sphereMesh.position.copy(path.points[j]);
                    scene.add(sphereMesh);
                    linkMeshes.push(sphereMesh);
                }
            }

            // 连杆标签（放在中间点）
            if (showLabels && path.midPoint) {
                const label = createLabel(`L${linkIndex}`, path.midPoint.clone().add(new THREE.Vector3(0, 0, radius * 2.5)), '#38bdf8');
                scene.add(label);
                labelMeshes.push(label);
            }
        }

        updateStatus();
    }

    function toggleIK(enabled) {
        ikEnabled = enabled;
        if (endEffectorMesh) {
            endEffectorMesh.visible = enabled && joints.length > 0;
        }
        if (transformControl) {
            if (enabled && joints.length > 0) {
                transformControl.attach(endEffectorMesh);
            } else {
                transformControl.detach();
            }
        }
    }

    function initScene() {
        const canvas = document.getElementById('canvas3d');
        const container = document.getElementById('canvas-container');

        // 场景
        scene = new THREE.Scene();
        
        // 渐变背景
        const bgColor = new THREE.Color(0x1a1a2e);
        scene.background = bgColor;
        scene.fog = new THREE.Fog(bgColor, 2000, 5000);

        // 相机
        camera = new THREE.PerspectiveCamera(
            50,
            container.clientWidth / container.clientHeight,
            1,
            10000
        );
        camera.position.set(600, 600, 800);
        camera.up.set(0, 0, 1);
        camera.lookAt(0, 0, 0);

        // 渲染器
        renderer = new THREE.WebGLRenderer({ 
            canvas: canvas, 
            antialias: true,
            alpha: true
        });
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;

        // 控制器
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.minDistance = 100;
        controls.maxDistance = 3000;
        
        // 鼠标按键映射：中键旋转，Ctrl+中键平移，滚轮缩放
        controls.mouseButtons = {
            LEFT: null,  // 左键不控制视角，用于IK交互
            MIDDLE: THREE.MOUSE.ROTATE,
            RIGHT: THREE.MOUSE.PAN
        };
        // Ctrl+中键平移
        controls.enablePan = true;

        // 光照
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
        scene.add(ambientLight);

        const mainLight = new THREE.DirectionalLight(0xffffff, 0.8);
        mainLight.position.set(500, 500, 1000);
        mainLight.castShadow = true;
        mainLight.shadow.mapSize.width = 2048;
        mainLight.shadow.mapSize.height = 2048;
        scene.add(mainLight);

        const fillLight = new THREE.DirectionalLight(0x4fc3f7, 0.3);
        fillLight.position.set(-500, -500, 500);
        scene.add(fillLight);

        // 网格地面
        const gridHelper = new THREE.GridHelper(1500, 30, 0x4a5568, 0x2d3748);
        gridHelper.rotation.x = Math.PI / 2;
        scene.add(gridHelper);

        // 世界坐标轴
        const worldAxes = new THREE.AxesHelper(120);
        scene.add(worldAxes);

        // CSS2D标签渲染器
        labelRenderer = new THREE.CSS2DRenderer();
        labelRenderer.setSize(container.clientWidth, container.clientHeight);
        labelRenderer.domElement.style.position = 'absolute';
        labelRenderer.domElement.style.top = '0';
        labelRenderer.domElement.style.pointerEvents = 'none';
        container.appendChild(labelRenderer.domElement);

        // 窗口调整
        window.addEventListener('resize', onWindowResize);
    }

    function onWindowResize() {
        const container = document.getElementById('canvas-container');
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
        labelRenderer.setSize(container.clientWidth, container.clientHeight);
    }

    function animate() {
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);
        labelRenderer.render(scene, camera);
    }

    // ========================================
    // 关节管理
    // ========================================

    function addJoint() {
        const id = jointIdCounter++;
        const joint = {
            id: id,
            type: 'R',  // R: 转动, P: 平动
            x: 0,       // 位置 (mm)
            y: 0,
            z: 100,     // 默认沿Z轴向上100mm
            rx: 0,      // 欧拉角 (度), ZYX顺序
            ry: 0,
            rz: 0,
            min: -180,  // 下限 (度 或 mm)
            max: 180,   // 上限 (度 或 mm)
            visible: true,  // 是否显示关节实体和坐标系
            linkMode: 'auto'  // 连杆延伸模式: 'auto' | 'x-axis' | 'y-axis' | 'z-axis'
        };
        joints.push(joint);
        jointValues.push(0);

        renderJointTable();
        updateVisualization();
        updateControlPanel();
    }

    function addDefaultJoint() {
        // 添加一个默认关节
        addJoint();
    }

    function deleteJoint(id) {
        const index = joints.findIndex(j => j.id === id);
        if (index !== -1) {
            joints.splice(index, 1);
            jointValues.splice(index, 1);
        }
        renderJointTable();
        updateVisualization();
        updateControlPanel();
    }

    function updateJointParam(id, param, value) {
        const joint = joints.find(j => j.id === id);
        if (joint) {
            joint[param] = parseFloat(value) || 0;
            debounce(updateVisualization, 50)();
        }
    }

    function updateJointType(id, type) {
        const joint = joints.find(j => j.id === id);
        if (joint) {
            joint.type = type;
            updateVisualization();
            updateControlPanel();
        }
    }

    function updateJointVisibility(id, visible) {
        const joint = joints.find(j => j.id === id);
        if (joint) {
            joint.visible = visible;
            updateVisualization();
        }
    }

    function updateJointLinkMode(id, mode) {
        const joint = joints.find(j => j.id === id);
        if (joint) {
            joint.linkMode = mode;
            updateVisualization();
        }
    }

    function renderJointTable() {
        const tbody = document.getElementById('joint-table-body');
        tbody.innerHTML = '';

        joints.forEach((joint, index) => {
            const tr = document.createElement('tr');
            tr.draggable = true;
            tr.dataset.index = index;
            tr.className = 'joint-row';
            
            const linkMode = joint.linkMode || 'auto';
            
            tr.innerHTML = `
                <td class="col-idx drag-handle" title="拖拽排序">${index + 1} ⋮⋮</td>
                <td>
                    <select onchange="RobotArm.updateJointType(${joint.id}, this.value)">
                        <option value="R" ${joint.type === 'R' ? 'selected' : ''}>转动</option>
                        <option value="P" ${joint.type === 'P' ? 'selected' : ''}>平动</option>
                    </select>
                </td>
                <td><input type="number" value="${joint.x}" step="1" 
                    onchange="RobotArm.updateJointParam(${joint.id}, 'x', this.value)"></td>
                <td><input type="number" value="${joint.y}" step="1" 
                    onchange="RobotArm.updateJointParam(${joint.id}, 'y', this.value)"></td>
                <td><input type="number" value="${joint.z}" step="1" 
                    onchange="RobotArm.updateJointParam(${joint.id}, 'z', this.value)"></td>
                <td><input type="number" value="${joint.rx}" step="1" 
                    onchange="RobotArm.updateJointParam(${joint.id}, 'rx', this.value)"></td>
                <td><input type="number" value="${joint.ry}" step="1" 
                    onchange="RobotArm.updateJointParam(${joint.id}, 'ry', this.value)"></td>
                <td><input type="number" value="${joint.rz}" step="1" 
                    onchange="RobotArm.updateJointParam(${joint.id}, 'rz', this.value)"></td>
                <td><input type="number" value="${joint.min !== undefined ? joint.min : -180}" step="1" 
                    onchange="RobotArm.updateJointParam(${joint.id}, 'min', this.value)"></td>
                <td><input type="number" value="${joint.max !== undefined ? joint.max : 180}" step="1" 
                    onchange="RobotArm.updateJointParam(${joint.id}, 'max', this.value)"></td>
                <td class="col-linkmode">
                    <select onchange="RobotArm.updateJointLinkMode(${joint.id}, this.value)" title="连杆延伸方式">
                        <option value="auto" ${linkMode === 'auto' ? 'selected' : ''}>自动</option>
                        <option value="x-axis" ${linkMode === 'x-axis' ? 'selected' : ''}>X轴</option>
                        <option value="y-axis" ${linkMode === 'y-axis' ? 'selected' : ''}>Y轴</option>
                        <option value="z-axis" ${linkMode === 'z-axis' ? 'selected' : ''}>Z轴</option>
                    </select>
                </td>
                <td class="col-visibility">
                    <label class="checkbox-label-inline">
                        <input type="checkbox" ${joint.visible !== false ? 'checked' : ''} 
                            onchange="RobotArm.updateJointVisibility(${joint.id}, this.checked)">
                        <span>显示</span>
                    </label>
                </td>
                <td>
                    <button class="delete-btn" onclick="RobotArm.deleteJoint(${joint.id})">✕</button>
                </td>
            `;
            
            // 添加拖拽事件监听器
            tr.addEventListener('dragstart', handleDragStart);
            tr.addEventListener('dragover', handleDragOver);
            tr.addEventListener('drop', handleDrop);
            tr.addEventListener('dragend', handleDragEnd);
            
            tbody.appendChild(tr);
        });

        document.getElementById('status-joints').textContent = `关节: ${joints.length}`;
    }

    // 拖拽相关变量
    let draggedRow = null;
    let draggedIndex = null;

    function handleDragStart(e) {
        draggedRow = this;
        draggedIndex = parseInt(this.dataset.index);
        this.style.opacity = '0.5';
        e.dataTransfer.effectAllowed = 'move';
        e.dataTransfer.setData('text/html', this.innerHTML);
    }

    function handleDragOver(e) {
        if (e.preventDefault) {
            e.preventDefault();
        }
        e.dataTransfer.dropEffect = 'move';
        
        const targetRow = this.closest('tr');
        if (targetRow && targetRow !== draggedRow && draggedRow) {
            const targetIndex = parseInt(targetRow.dataset.index);
            const draggedIndexNum = parseInt(draggedIndex);
            
            // 移除之前的drag-over样式
            document.querySelectorAll('.joint-row').forEach(row => {
                row.classList.remove('drag-over');
            });
            
            // 添加视觉反馈
            if (targetIndex < draggedIndexNum) {
                targetRow.classList.add('drag-over');
                targetRow.parentNode.insertBefore(draggedRow, targetRow);
            } else {
                targetRow.classList.add('drag-over');
                targetRow.parentNode.insertBefore(draggedRow, targetRow.nextSibling);
            }
        }
        return false;
    }

    function handleDrop(e) {
        if (e.stopPropagation) {
            e.stopPropagation();
        }
        
        e.preventDefault();
        
        if (draggedRow) {
            // 根据当前DOM中的行顺序重新排列数组
            const rows = Array.from(document.querySelectorAll('#joint-table-body tr'));
            const newJoints = [];
            const newJointValues = [];
            
            rows.forEach((row) => {
                const oldIndex = parseInt(row.dataset.index);
                newJoints.push(joints[oldIndex]);
                newJointValues.push(jointValues[oldIndex]);
            });
            
            joints = newJoints;
            jointValues = newJointValues;
            
            // 重新渲染表格和可视化
            renderJointTable();
            updateVisualization();
            updateControlPanel();
        }
        
        return false;
    }

    function handleDragEnd(e) {
        this.style.opacity = '';
        // 移除所有drag-over样式
        document.querySelectorAll('.joint-row').forEach(row => {
            row.classList.remove('drag-over');
        });
        draggedRow = null;
        draggedIndex = null;
    }

    // ========================================
    // 正向运动学
    // ========================================

    /**
     * 构建关节的基础变换矩阵（关节变量=0时）
     * T_base = Trans(x,y,z) * RotZ(rz) * RotY(ry) * RotX(rx)
     */
    function buildBaseTransform(joint) {
        const T = new THREE.Matrix4();
        
        // 平移
        const trans = new THREE.Matrix4().makeTranslation(joint.x, joint.y, joint.z);
        
        // 欧拉角旋转 (ZYX顺序，即先绕X，再绕Y，最后绕Z)
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
     * 构建关节变量引起的变换
     * 对于R关节: Rz(θ)
     * 对于P关节: Tz(d)
     */
    function buildJointTransform(joint, value) {
        if (joint.type === 'R') {
            return new THREE.Matrix4().makeRotationZ(value);
        } else {
            return new THREE.Matrix4().makeTranslation(0, 0, value);
        }
    }

    /**
     * 计算完整的正向运动学
     * 返回每个关节坐标系的变换矩阵
     */
    function computeFK(jointVals) {
        const transforms = [];
        const positions = [new THREE.Vector3(0, 0, 0)]; // 基座位置
        const xAxes = [new THREE.Vector3(1, 0, 0)]; // 基座X轴
        const yAxes = [new THREE.Vector3(0, 1, 0)]; // 基座Y轴
        const zAxes = [new THREE.Vector3(0, 0, 1)]; // 基座Z轴
        
        let T_world = new THREE.Matrix4(); // 累积变换

        for (let i = 0; i < joints.length; i++) {
            const joint = joints[i];
            const value = jointVals[i];

            // 基础变换 + 关节变量变换
            const T_base = buildBaseTransform(joint);
            const T_joint = buildJointTransform(joint, value);
            
            // 局部变换 = 基础变换 * 关节变换
            const T_local = T_base.clone().multiply(T_joint);
            
            // 累积到世界坐标系
            T_world = T_world.clone().multiply(T_local);

            // 提取位置
            const pos = new THREE.Vector3();
            pos.setFromMatrixPosition(T_world);
            positions.push(pos);

            // 提取X轴方向
            const xAxis = new THREE.Vector3(1, 0, 0);
            xAxis.applyMatrix4(T_world.clone().setPosition(0, 0, 0));
            xAxes.push(xAxis);

            // 提取Y轴方向
            const yAxis = new THREE.Vector3(0, 1, 0);
            yAxis.applyMatrix4(T_world.clone().setPosition(0, 0, 0));
            yAxes.push(yAxis);

            // 提取Z轴方向
            const zAxis = new THREE.Vector3(0, 0, 1);
            zAxis.applyMatrix4(T_world.clone().setPosition(0, 0, 0));
            zAxes.push(zAxis);

            transforms.push({
                matrix: T_world.clone(),
                position: pos.clone(),
                xAxis: xAxis.clone(),
                yAxis: yAxis.clone(),
                zAxis: zAxis.clone(),
                type: joint.type
            });
        }

        return { transforms, positions, xAxes, yAxes, zAxes };
    }

    /**
     * 仅计算末端位置（用于工作空间渲染）
     */
    function computeEndPosition(jointVals) {
        let T = new THREE.Matrix4();
        
        for (let i = 0; i < joints.length; i++) {
            const joint = joints[i];
            const T_base = buildBaseTransform(joint);
            const T_joint = buildJointTransform(joint, jointVals[i]);
            T = T.clone().multiply(T_base).multiply(T_joint);
        }
        
        const pos = new THREE.Vector3();
        pos.setFromMatrixPosition(T);
        return pos;
    }

    /**
     * 判断两条直线是否相交（近似）
     * 返回 {intersects: boolean, point: Vector3（如果相交）, distance: 最短距离}
     */
    function computeLineIntersection(p1, dir1, p2, dir2, tolerance = 1.0) {
        // 归一化方向向量
        const d1 = dir1.clone().normalize();
        const d2 = dir2.clone().normalize();
        
        // 检查是否平行
        const cross = new THREE.Vector3().crossVectors(d1, d2);
        const crossLen = cross.length();
        
        if (crossLen < 0.01) {
            // 平行或重合
            return { intersects: false, parallel: true, distance: Infinity };
        }
        
        // 计算两直线的最近点
        // 参数方程: L1 = p1 + t*d1, L2 = p2 + s*d2
        // 最小距离点满足: (p1 + t*d1 - p2 - s*d2) · d1 = 0
        //                  (p1 + t*d1 - p2 - s*d2) · d2 = 0
        
        const w = new THREE.Vector3().subVectors(p1, p2);
        const a = d1.dot(d1);
        const b = d1.dot(d2);
        const c = d2.dot(d2);
        const d = d1.dot(w);
        const e = d2.dot(w);
        
        const denom = a * c - b * b;
        const t = (b * e - c * d) / denom;
        const s = (a * e - b * d) / denom;
        
        const point1 = p1.clone().add(d1.clone().multiplyScalar(t));
        const point2 = p2.clone().add(d2.clone().multiplyScalar(s));
        
        const distance = point1.distanceTo(point2);
        
        if (distance < tolerance) {
            // 相交（或足够接近）
            const midPoint = point1.clone().add(point2).multiplyScalar(0.5);
            return { intersects: true, parallel: false, point: midPoint, distance, t, s };
        } else {
            // 异面直线
            return { intersects: false, parallel: false, distance, point1, point2 };
        }
    }

    /**
     * 计算连杆的中间路径点
     * 根据关节轴线关系和手动设置的连杆模式，决定是直线连接还是折线连接
     */
    function computeLinkPath(posStart, xAxisStart, yAxisStart, zAxisStart, posEnd, zAxisEnd, linkMode = 'auto') {
        // 如果手动指定了延伸轴
        if (linkMode === 'x-axis' || linkMode === 'y-axis' || linkMode === 'z-axis') {
            let firstAxis;
            switch (linkMode) {
                case 'x-axis':
                    firstAxis = xAxisStart;
                    break;
                case 'y-axis':
                    firstAxis = yAxisStart;
                    break;
                case 'z-axis':
                    firstAxis = zAxisStart;
                    break;
            }
            
            // 先沿指定轴方向，再沿终点Z轴方向
            const toEnd = new THREE.Vector3().subVectors(posEnd, posStart);
            const axisProj = toEnd.dot(firstAxis);
            const midPoint = posStart.clone().add(firstAxis.clone().multiplyScalar(axisProj));
            
            return {
                type: linkMode,
                points: [posStart, midPoint, posEnd],
                midPoint: midPoint
            };
        }
        
        // 自动模式：检查轴线关系
        const intersection = computeLineIntersection(posStart, zAxisStart, posEnd, zAxisEnd, 2.0);
        
        if (intersection.intersects) {
            // 轴线相交：先沿起点关节轴线方向，再沿终点关节轴线方向
            const midPoint = intersection.point;
            return {
                type: 'intersecting',
                points: [posStart, midPoint, posEnd],
                midPoint: midPoint
            };
        } else if (intersection.parallel) {
            // 轴线平行：先沿起点关节X轴方向，再沿终点关节轴线方向
            const toEnd = new THREE.Vector3().subVectors(posEnd, posStart);
            const xProj = toEnd.dot(xAxisStart);
            const midPoint = posStart.clone().add(xAxisStart.clone().multiplyScalar(xProj));
            
            return {
                type: 'parallel',
                points: [posStart, midPoint, posEnd],
                midPoint: midPoint
            };
        } else {
            // 异面直线：使用平行情况的处理方式（先沿X轴，再沿Z轴）
            const toEnd = new THREE.Vector3().subVectors(posEnd, posStart);
            const xProj = toEnd.dot(xAxisStart);
            const midPoint = posStart.clone().add(xAxisStart.clone().multiplyScalar(xProj));
            
            return {
                type: 'skew',
                points: [posStart, midPoint, posEnd],
                midPoint: midPoint
            };
        }
    }

    /**
     * 创建圆柱体连杆
     */
    function createLinkCylinder(start, end, radius, material) {
        const dir = new THREE.Vector3().subVectors(end, start);
        const length = dir.length();
        
        if (length < 0.1) return null;
        
        const geom = new THREE.CylinderGeometry(radius, radius, length, 16);
        const mesh = new THREE.Mesh(geom, material);
        
        const mid = start.clone().add(dir.clone().multiplyScalar(0.5));
        mesh.position.copy(mid);
        
        const axis = new THREE.Vector3(0, 1, 0);
        const quat = new THREE.Quaternion();
        quat.setFromUnitVectors(axis, dir.clone().normalize());
        mesh.setRotationFromQuaternion(quat);
        
        return mesh;
    }

    // ========================================
    // MDH参数导出
    // ========================================

    /**
     * 从物理结构导出MDH参数
     * 
     * MDH变换: T = Rx(α) * Tx(a) * Rz(θ) * Tz(d)
     * 
     * 给定从{i-1}到{i}的变换矩阵T，需要分解出α, a, d, θ
     * 这里假设关节轴是局部Z轴
     */
    function extractMDHParams() {
        if (joints.length === 0) return [];

        const mdhParams = [];
        let T_prev = new THREE.Matrix4(); // 上一个坐标系的累积变换

        for (let i = 0; i < joints.length; i++) {
            const joint = joints[i];
            
            // 计算从{i-1}到{i}的变换（关节变量=0时）
            const T_base = buildBaseTransform(joint);
            
            // 提取MDH参数
            // T_base 应该等于 Rx(α) * Tx(a) * Rz(θ) * Tz(d)
            // 我们需要从T_base中分解出这些参数
            
            const params = decomposeToMDH(T_base, joint.type);
            mdhParams.push({
                index: i + 1,
                type: joint.type,
                alpha: params.alpha,
                a: params.a,
                d: params.d,
                theta: params.theta,
                variable: joint.type === 'R' ? 'θ' : 'd'
            });
        }

        return mdhParams;
    }

    /**
     * 将变换矩阵分解为MDH参数
     * 
     * 这是一个复杂的逆问题，需要根据MDH约定进行分解
     * T = Rx(α) * Tx(a) * Rz(θ) * Tz(d)
     */
    function decomposeToMDH(T, jointType) {
        // 提取旋转矩阵和平移向量
        const R = new THREE.Matrix3().setFromMatrix4(T);
        const p = new THREE.Vector3();
        p.setFromMatrixPosition(T);

        // 从旋转矩阵中提取角度
        // R = Rx(α) * Rz(θ)
        // 
        // Rx(α) = [1,    0,      0   ]
        //         [0,  cos(α), -sin(α)]
        //         [0,  sin(α),  cos(α)]
        //
        // Rz(θ) = [cos(θ), -sin(θ), 0]
        //         [sin(θ),  cos(θ), 0]
        //         [0,       0,      1]
        //
        // R = Rx * Rz = [cos(θ),       -sin(θ),        0      ]
        //               [cos(α)sin(θ),  cos(α)cos(θ), -sin(α) ]
        //               [sin(α)sin(θ),  sin(α)cos(θ),  cos(α) ]

        const elements = R.elements;
        // Three.js Matrix3 是列优先存储
        // elements[0] = R[0,0], elements[1] = R[1,0], elements[2] = R[2,0]
        // elements[3] = R[0,1], elements[4] = R[1,1], elements[5] = R[2,1]
        // elements[6] = R[0,2], elements[7] = R[1,2], elements[8] = R[2,2]

        const r00 = elements[0], r10 = elements[1], r20 = elements[2];
        const r01 = elements[3], r11 = elements[4], r21 = elements[5];
        const r02 = elements[6], r12 = elements[7], r22 = elements[8];

        // 从R[2,2] = cos(α) 得到 α
        // 从R[1,2] = -sin(α) 得到 α 的符号
        let alpha = Math.atan2(-r12, r22);

        // 从R[0,0] = cos(θ), R[1,0] = cos(α)sin(θ) 或 R[0,1] = -sin(θ) 得到 θ
        let theta;
        if (Math.abs(Math.cos(alpha)) > 0.001) {
            theta = Math.atan2(r10 / Math.cos(alpha), r00);
        } else {
            // cos(α) ≈ 0, 即 α ≈ ±90°
            // 此时用 R[2,0] = sin(α)sin(θ), R[2,1] = sin(α)cos(θ)
            theta = Math.atan2(r20 / Math.sin(alpha), r21 / Math.sin(alpha));
        }

        // 对于MDH，平移参数的计算
        // p = Rx(α) * [a, 0, d]^T
        // p_x = a
        // p_y = -d * sin(α)
        // p_z = d * cos(α)
        
        // 但这是在旋转前的坐标，实际上：
        // 完整变换 T = Rx(α) * Tx(a) * Rz(θ) * Tz(d)
        // 位置 p = Rx(α) * Rz(θ) * [0,0,d]^T + Rx(α) * [a,0,0]^T
        //        = R * [0,0,d]^T + Rx(α) * [a,0,0]^T
        
        // 简化处理：假设θ对位置的影响已经包含在T_base中（关节变量=0时θ=0）
        // 当θ=0时：p = Rx(α) * [a, 0, d]^T
        // p_x = a
        // p_y = -d * sin(α)
        // p_z = d * cos(α)
        
        let a, d;
        if (Math.abs(Math.cos(alpha)) > 0.001) {
            a = p.x;
            d = p.z / Math.cos(alpha);
        } else {
            a = p.x;
            d = -p.y / Math.sin(alpha);
        }

        // 对于关节变量，θ或d是变量
        // 如果是转动关节，theta是初始值（通常为0）
        // 如果是平动关节，d是初始值（通常为0），需要从提取的d中减去

        return {
            alpha: alpha * 180 / Math.PI,  // 转换为度
            a: a,
            d: jointType === 'P' ? 0 : d,  // 平动关节的d是变量，初始值设为0
            theta: jointType === 'R' ? 0 : theta * 180 / Math.PI  // 转动关节的θ是变量
        };
    }

    function exportMDH() {
        if (joints.length === 0) {
            alert('请先添加关节！');
            return;
        }

        const mdhParams = extractMDHParams();
        
        // 显示MDH参数表
        const section = document.getElementById('mdh-section');
        section.style.display = 'block';
        
        const tbody = document.getElementById('mdh-table-body');
        tbody.innerHTML = '';
        
        const unit = document.getElementById('angle-unit').value;
        
        mdhParams.forEach(param => {
            const tr = document.createElement('tr');
            const alphaVal = unit === 'degree' ? param.alpha.toFixed(2) + '°' : (param.alpha * Math.PI / 180).toFixed(4);
            const thetaVal = param.type === 'R' 
                ? `<span class="variable">θ${param.index}</span>` 
                : (unit === 'degree' ? param.theta.toFixed(2) + '°' : (param.theta * Math.PI / 180).toFixed(4));
            const dVal = param.type === 'P' 
                ? `<span class="variable">d${param.index}</span>` 
                : param.d.toFixed(2);
            
            tr.innerHTML = `
                <td>${param.index}</td>
                <td>${alphaVal}</td>
                <td>${param.a.toFixed(2)}</td>
                <td>${dVal}</td>
                <td>${thetaVal}</td>
            `;
            tbody.appendChild(tr);
        });

        // 滚动到MDH表格
        section.scrollIntoView({ behavior: 'smooth' });
    }

    // ========================================
    // 可视化
    // ========================================

    function updateVisualization() {
        if (!scene) return;

        clearVisualization();

        if (joints.length === 0) {
            updateStatus();
            return;
        }

        const radius = parseFloat(document.getElementById('link-radius').value) || 25;
        const axisSize = parseFloat(document.getElementById('axis-size').value) || 3;
        const jointDiameter = parseFloat(document.getElementById('joint-diameter').value) || 60;
        const jointHeight = parseFloat(document.getElementById('joint-height').value) || 60;
        const showAxes = document.getElementById('show-axes').checked;
        const showLabels = document.getElementById('show-labels').checked;

        // 计算FK
        const fk = computeFK(jointValues);
        const { transforms, positions, xAxes, yAxes, zAxes } = fk;

        // 材质
        const jointMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xef4444,
            shininess: 80
        });
        const linkMaterial = new THREE.MeshPhongMaterial({ 
            color: 0x38bdf8,
            shininess: 60,
            transparent: true,
            opacity: 0.85
        });
        const baseMaterial = new THREE.MeshPhongMaterial({ 
            color: 0x10b981,
            shininess: 80
        });

        // 绘制基座
        const baseGeom = new THREE.CylinderGeometry(radius * 2.5, radius * 3, radius * 2, 32);
        const baseMesh = new THREE.Mesh(baseGeom, baseMaterial);
        baseMesh.rotation.x = Math.PI / 2;
        baseMesh.position.z = -radius;
        scene.add(baseMesh);
        jointMeshes.push(baseMesh);

        // 绘制关节和连杆
        const jointRadius = jointDiameter / 2;

        // 基座标签
        if (showLabels && positions.length > 0) {
            const basePos = positions[0];
            const label = createLabel('基座{0}', basePos.clone().add(new THREE.Vector3(0, 0, radius * 3)), '#10b981');
            scene.add(label);
            labelMeshes.push(label);
        }

        positions.forEach((pos, i) => {
            if (i === 0) {
                // 基座保持原样（已经是圆柱体）
                return;
            }
            
            // 检查关节是否可见（positions[i]对应joints[i-1]）
            const joint = joints[i - 1];
            const isVisible = joint.visible !== false;
            
            if (isVisible) {
                // 创建圆柱体关节
                const jointGeom = new THREE.CylinderGeometry(jointRadius, jointRadius, jointHeight, 32);
                const jointMesh = new THREE.Mesh(jointGeom, jointMaterial);
                
                // 获取关节Z轴方向（关节轴向）
                const zAxis = zAxes[i].clone().normalize();
                
                // 将圆柱体沿Z轴方向放置
                // Three.js的CylinderGeometry默认沿Y轴，需要旋转到Z轴
                const up = new THREE.Vector3(0, 1, 0);
                const quat = new THREE.Quaternion();
                quat.setFromUnitVectors(up, zAxis);
                jointMesh.setRotationFromQuaternion(quat);
                
                // 设置位置
                jointMesh.position.copy(pos);
                scene.add(jointMesh);
                jointMeshes.push(jointMesh);

                // 标签
                if (showLabels) {
                    const labelText = `{${i}}`;
                    const labelColor = '#f1f5f9';
                    // 标签位置：在圆柱体上方，沿Z轴方向偏移
                    const labelOffset = zAxis.clone().multiplyScalar(jointHeight / 2 + jointRadius + 5);
                    const label = createLabel(labelText, pos.clone().add(labelOffset), labelColor);
                    scene.add(label);
                    labelMeshes.push(label);
                }
            }

            // 坐标系显示（需要同时满足全局显示和关节可见）
            if (showAxes && isVisible && i > 0) {
                const transform = transforms[i - 1];
                const axesHelper = new THREE.AxesHelper(radius * 6 * axisSize);
                axesHelper.position.copy(pos);
                
                const rotation = new THREE.Matrix4();
                rotation.extractRotation(transform.matrix);
                axesHelper.setRotationFromMatrix(rotation);
                
                scene.add(axesHelper);
                axisMeshes.push(axesHelper);

                // 关节轴箭头
                const axisColor = transform.type === 'R' ? 0xff00ff : 0xffa500;
                const arrow = new THREE.ArrowHelper(
                    zAxes[i].clone().normalize(),
                    pos,
                    radius * 8 * axisSize,
                    axisColor,
                    radius * 2 * axisSize,
                    radius * axisSize
                );
                scene.add(arrow);
                axisMeshes.push(arrow);
            }

            // 基座坐标轴
            if (showAxes && i === 0) {
                const axesHelper = new THREE.AxesHelper(radius * 8 * axisSize);
                axesHelper.position.copy(pos);
                scene.add(axesHelper);
                axisMeshes.push(axesHelper);
            }
        });

        // 绘制连杆
        // MDH约定：关节i连接连杆i-1和连杆i
        // - positions[0] = 基座原点
        // - positions[i] = 关节i位置 (i >= 1)
        // - 连杆0（基座到关节1）不渲染
        // - 连杆i = 从关节i到关节i+1的线段 (i >= 1)
        // 所以从i=2开始，渲染 positions[i-1] 到 positions[i] 的连杆（编号为i-1）
        for (let i = 2; i < positions.length; i++) {
            const startPos = positions[i - 1];
            const endPos = positions[i];
            const linkIndex = i - 1; // 连杆编号
            
            // 获取关节坐标系信息
            const startTransform = transforms[i - 2];
            const endTransform = transforms[i - 1];
            
            const xAxisStart = startTransform.xAxis.clone().normalize();
            const yAxisStart = startTransform.yAxis.clone().normalize();
            const zAxisStart = startTransform.zAxis.clone().normalize();
            const zAxisEnd = endTransform.zAxis.clone().normalize();
            
            // 获取该关节的连杆模式
            const startJoint = joints[i - 2];
            const linkMode = startJoint.linkMode || 'auto';
            
            // 计算连杆路径
            const path = computeLinkPath(startPos, xAxisStart, yAxisStart, zAxisStart, endPos, zAxisEnd, linkMode);
            
            // 绘制连杆段
            for (let j = 0; j < path.points.length - 1; j++) {
                const segStart = path.points[j];
                const segEnd = path.points[j + 1];
                const mesh = createLinkCylinder(segStart, segEnd, radius, linkMaterial);
                
                if (mesh) {
                    scene.add(mesh);
                    linkMeshes.push(mesh);
                }
            }
            
            // 在转折点添加连接球体
            if (path.points.length > 2) {
                for (let j = 1; j < path.points.length - 1; j++) {
                    const sphereGeom = new THREE.SphereGeometry(radius, 16, 16);
                    const sphereMesh = new THREE.Mesh(sphereGeom, linkMaterial);
                    sphereMesh.position.copy(path.points[j]);
                    scene.add(sphereMesh);
                    linkMeshes.push(sphereMesh);
                }
            }

            // 连杆标签（放在中间点）
            if (showLabels && path.midPoint) {
                const label = createLabel(`L${linkIndex}`, path.midPoint.clone().add(new THREE.Vector3(0, 0, radius * 2.5)), '#38bdf8');
                scene.add(label);
                labelMeshes.push(label);
            }
        }

        updateStatus();
        updateEndEffectorPosition();
    }

    function clearVisualization() {
        jointMeshes.forEach(m => scene.remove(m));
        linkMeshes.forEach(m => scene.remove(m));
        axisMeshes.forEach(m => scene.remove(m));
        labelMeshes.forEach(l => {
            if (l.element && l.element.parentNode) {
                l.element.parentNode.removeChild(l.element);
            }
            scene.remove(l);
        });
        
        jointMeshes = [];
        linkMeshes = [];
        axisMeshes = [];
        labelMeshes = [];
    }

    function createLabel(text, position, color = '#f1f5f9') {
        const div = document.createElement('div');
        div.textContent = text;
        div.style.cssText = `
            color: ${color};
            font-family: 'JetBrains Mono', monospace;
            font-size: 12px;
            font-weight: 600;
            background: rgba(30, 41, 59, 0.9);
            padding: 2px 6px;
            border-radius: 4px;
            border: 1px solid rgba(100, 116, 139, 0.5);
            pointer-events: none;
        `;
        
        const label = new THREE.CSS2DObject(div);
        label.position.copy(position);
        label.element = div;
        return label;
    }

    function updateStatus() {
        const endPos = joints.length > 0 ? computeEndPosition(jointValues) : new THREE.Vector3();
        document.getElementById('status-pos').textContent = 
            `末端: (${endPos.x.toFixed(1)}, ${endPos.y.toFixed(1)}, ${endPos.z.toFixed(1)})`;
    }

    // ========================================
    // 关节控制面板
    // ========================================

    function updateControlPanel() {
        const container = document.getElementById('sliders-container');
        container.innerHTML = '';

        if (joints.length === 0) return;

        const unit = document.getElementById('angle-unit').value;

        joints.forEach((joint, i) => {
            const div = document.createElement('div');
            div.className = 'slider-group';
            
            let min, max, step, displayValue, unitStr;
            if (joint.type === 'R') {
                // 使用关节的限位值，如果没有则使用默认值
                const minLimit = joint.min !== undefined ? joint.min : -180;
                const maxLimit = joint.max !== undefined ? joint.max : 180;
                
                min = unit === 'degree' ? minLimit : minLimit * Math.PI / 180;
                max = unit === 'degree' ? maxLimit : maxLimit * Math.PI / 180;
                step = unit === 'degree' ? 1 : 0.01;
                displayValue = unit === 'degree' 
                    ? (jointValues[i] * 180 / Math.PI).toFixed(1)
                    : jointValues[i].toFixed(3);
                unitStr = unit === 'degree' ? '°' : 'rad';
            } else {
                // 平动关节的限位值
                min = joint.min !== undefined ? joint.min : -500;
                max = joint.max !== undefined ? joint.max : 500;
                step = 1;
                displayValue = jointValues[i].toFixed(1);
                unitStr = 'mm';
            }

            const sliderValue = joint.type === 'R' && unit === 'degree' 
                ? jointValues[i] * 180 / Math.PI 
                : jointValues[i];

            div.innerHTML = `
                <label>
                    <span>关节 ${i + 1} (${joint.type === 'R' ? 'θ' : 'd'})</span>
                    <span class="slider-value">${displayValue}${unitStr}</span>
                </label>
                <input type="range" min="${min}" max="${max}" step="${step}" value="${sliderValue}"
                    oninput="RobotArm.onSliderChange(${i}, this.value)">
            `;
            container.appendChild(div);
        });
    }

    function onSliderChange(index, value) {
        const joint = joints[index];
        const unit = document.getElementById('angle-unit').value;
        
        let realValue = parseFloat(value);
        if (joint.type === 'R' && unit === 'degree') {
            realValue = realValue * Math.PI / 180;
        }
        
        jointValues[index] = realValue;
        
        // 更新显示值
        const slider = document.querySelectorAll('.slider-group')[index];
        const valueSpan = slider.querySelector('.slider-value');
        if (joint.type === 'R') {
            const displayVal = unit === 'degree' 
                ? (realValue * 180 / Math.PI).toFixed(1) 
                : realValue.toFixed(3);
            valueSpan.textContent = displayVal + (unit === 'degree' ? '°' : 'rad');
        } else {
            valueSpan.textContent = realValue.toFixed(1) + 'mm';
        }
        
        updateVisualization();
    }

    function resetJoints() {
        jointValues = joints.map(() => 0);
        updateControlPanel();
        updateVisualization();
    }

    // ========================================
    // 操作说明折叠
    // ========================================

    function toggleInstructions() {
        const content = document.getElementById('instructions-content');
        const icon = document.getElementById('collapse-icon');
        
        if (content.style.display === 'none') {
            content.style.display = 'block';
            icon.textContent = '▼';
        } else {
            content.style.display = 'none';
            icon.textContent = '▶';
        }
    }

    // ========================================
    // 工作空间渲染
    // ========================================

    async function renderWorkspace() {
        if (isRendering) {
            alert('正在渲染中...');
            return;
        }

        if (joints.length === 0) {
            alert('请先添加关节！');
            return;
        }

        // 不自动清除，允许叠加渲染
        isRendering = true;
        cancelRendering = false;

        // 读取参数
        const computeDexterous = document.getElementById('compute-dexterous').checked;
        const showGradient = document.getElementById('show-gradient').checked;
        const dexterousThreshold = parseFloat(document.getElementById('dexterous-threshold').value) || 0.5;

        // 体积显示初始化
        const volumeBox = document.getElementById('workspace-volume');
        const reachableVolumeVal = document.getElementById('reachable-volume-value');
        const dexterousInfo = document.getElementById('dexterous-info');
        const dexterousVolumeVal = document.getElementById('dexterous-volume-value');
        const dexterousPercentageVal = document.getElementById('dexterous-percentage-value');
        
        if (volumeBox && reachableVolumeVal) {
            volumeBox.style.display = 'block';
            reachableVolumeVal.textContent = '计算中...';
            if (dexterousInfo) {
                dexterousInfo.style.display = computeDexterous ? 'block' : 'none';
                if (computeDexterous) {
                    dexterousVolumeVal.textContent = '计算中...';
                    dexterousPercentageVal.textContent = '计算中...';
                }
            }
        }

        const btn = document.getElementById('render-workspace-btn');
        btn.textContent = '⏸️ 取消';
        btn.onclick = () => { cancelRendering = true; };

        const progressContainer = document.getElementById('progress-container');
        const progressBar = document.getElementById('progress-bar');
        const progressText = document.getElementById('progress-text');
        progressContainer.style.display = 'block';

        const targetSamples = parseInt(document.getElementById('workspace-samples').value) || 1000000;
        const voxelSize = Math.max(1, parseFloat(document.getElementById('voxel-size').value) || 50);
        const reachablePoints = [];
        const dexterityData = []; // 存储{point, dexterity}
        const batchSize = 1000;

        try {
            for (let i = 0; i < targetSamples && !cancelRendering; i += batchSize) {
                const batch = Math.min(batchSize, targetSamples - i);
                
                for (let j = 0; j < batch; j++) {
                    const config = joints.map(joint => {
                        if (joint.type === 'R') {
                            // 使用关节配置的限位（转换为弧度）
                            const minLimit = joint.min !== undefined ? joint.min * Math.PI / 180 : -Math.PI;
                            const maxLimit = joint.max !== undefined ? joint.max * Math.PI / 180 : Math.PI;
                            return minLimit + Math.random() * (maxLimit - minLimit);
                        } else {
                            // 使用关节配置的限位
                            const minLimit = joint.min !== undefined ? joint.min : -500;
                            const maxLimit = joint.max !== undefined ? joint.max : 500;
                            return minLimit + Math.random() * (maxLimit - minLimit);
                        }
                    });
                    
                    const pos = computeEndPosition(config);
                    reachablePoints.push(pos);
                }

                const progress = Math.round(((i + batch) / targetSamples) * 100);
                progressBar.style.width = progress + '%';
                progressText.textContent = `采样: ${progress}%`;

                await new Promise(r => setTimeout(r, 0));
            }

            // 如果需要计算灵活度，进行第二阶段分析
            if (computeDexterous && !cancelRendering && reachablePoints.length > 0) {
                progressText.textContent = '分析灵活度分布...';
                progressBar.style.width = '0%';
                
                // 对可达空间的点进行体素化，减少需要测试的点数
                const voxelMap = new Map();
                const inv = 1 / voxelSize;
                
                for (let i = 0; i < reachablePoints.length; i++) {
                    const p = reachablePoints[i];
                    const key = `${Math.floor(p.x * inv)},${Math.floor(p.y * inv)},${Math.floor(p.z * inv)}`;
                    if (!voxelMap.has(key)) {
                        voxelMap.set(key, p.clone());
                    }
                }
                
                const uniquePoints = Array.from(voxelMap.values());
                progressText.textContent = `计算 ${uniquePoints.length} 个体素的灵活度...`;
                
                // 第一阶段：计算所有点的原始可操作度，找到最大值
                progressText.textContent = '第1阶段: 计算可操作度范围...';
                const rawManipValues = [];
                for (let i = 0; i < uniquePoints.length && !cancelRendering; i++) {
                    const point = uniquePoints[i];
                    const result = computeDexterityIndex(point, 1); // 第一阶段：只用于获取原始可操作度
                    rawManipValues.push(result.rawManip);
                    
                    if (i % 100 === 0) {
                        const progress = Math.round((i / uniquePoints.length) * 50);
                        progressBar.style.width = progress + '%';
                        progressText.textContent = `第1阶段: ${progress}%`;
                        await new Promise(r => setTimeout(r, 0));
                    }
                }
                
                const maxManipulability = Math.max(...rawManipValues, 1e-10);
                console.log('最大可操作度:', maxManipulability);
                
                // 第二阶段：计算综合灵活度指数（使用固定权重0.4和0.6）
                progressText.textContent = '第2阶段: 计算综合灵活度...';
                for (let i = 0; i < uniquePoints.length && !cancelRendering; i++) {
                    const point = uniquePoints[i];
                    const result = computeDexterityIndex(point, maxManipulability);
                    
                    if (result.dexterity > 0) {
                        dexterityData.push({
                            point: point,
                            dexterity: result.dexterity,
                            manipulability: result.manipulability,
                            isotropy: result.isotropy
                        });
                    }
                    
                    if (i % 50 === 0) {
                        const progress = 50 + Math.round((i / uniquePoints.length) * 50);
                        progressBar.style.width = progress + '%';
                        progressText.textContent = `第2阶段: ${progress - 50}% (${dexterityData.length}/${i+1})`;
                        await new Promise(r => setTimeout(r, 0));
                    }
                }
            }

            if (!cancelRendering && reachablePoints.length > 0) {
                // 计算Z范围
                let minZ = Infinity, maxZ = -Infinity;
                reachablePoints.forEach(p => {
                    minZ = Math.min(minZ, p.z);
                    maxZ = Math.max(maxZ, p.z);
                });
                const zRange = maxZ - minZ || 1;

                // 创建可达空间点云（蓝色渐变）
                const reachableGeometry = new THREE.BufferGeometry();
                const reachablePositions = new Float32Array(reachablePoints.length * 3);
                const reachableColors = new Float32Array(reachablePoints.length * 3);

                reachablePoints.forEach((p, i) => {
                    reachablePositions[i * 3] = p.x;
                    reachablePositions[i * 3 + 1] = p.y;
                    reachablePositions[i * 3 + 2] = p.z;

                    const hue = (1 - (p.z - minZ) / zRange) * 0.6 + 0.5; // 蓝-青色
                    const color = new THREE.Color();
                    color.setHSL(hue, 0.8, 0.5);
                    reachableColors[i * 3] = color.r;
                    reachableColors[i * 3 + 1] = color.g;
                    reachableColors[i * 3 + 2] = color.b;
                });

                reachableGeometry.setAttribute('position', new THREE.BufferAttribute(reachablePositions, 3));
                reachableGeometry.setAttribute('color', new THREE.BufferAttribute(reachableColors, 3));

                const reachableMaterial = new THREE.PointsMaterial({
                    size: reachablePoints.length > 1000000 ? 1 : 2,
                    vertexColors: true,
                    transparent: true,
                    opacity: 0.5
                });

                const reachableCloud = new THREE.Points(reachableGeometry, reachableMaterial);
                scene.add(reachableCloud);
                workspacePoints.push(reachableCloud);

                // 如果有灵活度数据，创建灵活度可视化点云
                if (dexterityData.length > 0) {
                    const dexterousGeometry = new THREE.BufferGeometry();
                    const dexterousPositions = new Float32Array(dexterityData.length * 3);
                    const dexterousColors = new Float32Array(dexterityData.length * 3);

                    dexterityData.forEach((data, i) => {
                        const p = data.point;
                        dexterousPositions[i * 3] = p.x;
                        dexterousPositions[i * 3 + 1] = p.y;
                        dexterousPositions[i * 3 + 2] = p.z;

                        if (showGradient) {
                            // 渐变色：红色(低灵活度) -> 黄色 -> 绿色(高灵活度)
                            // dexterity: 0-1
                            let hue, saturation, lightness;
                            if (data.dexterity < 0.5) {
                                // 红到黄：hue 0° -> 60°
                                hue = data.dexterity * 0.334; // 0 -> 0.167
                                saturation = 0.9;
                                lightness = 0.4 + data.dexterity * 0.1;
                            } else {
                                // 黄到绿：hue 60° -> 120°
                                hue = 0.167 + ((data.dexterity - 0.5) * 0.334); // 0.167 -> 0.334
                                saturation = 0.85;
                                lightness = 0.45 + (data.dexterity - 0.5) * 0.1;
                            }
                            
                            const color = new THREE.Color();
                            color.setHSL(hue, saturation, lightness);
                            dexterousColors[i * 3] = color.r;
                            dexterousColors[i * 3 + 1] = color.g;
                            dexterousColors[i * 3 + 2] = color.b;
                        } else {
                            // 单色高亮（绿色）- 强度与灵活度成正比
                            const intensity = 0.3 + data.dexterity * 0.7;
                            dexterousColors[i * 3] = 0;
                            dexterousColors[i * 3 + 1] = intensity;
                            dexterousColors[i * 3 + 2] = 0;
                        }
                    });

                    dexterousGeometry.setAttribute('position', new THREE.BufferAttribute(dexterousPositions, 3));
                    dexterousGeometry.setAttribute('color', new THREE.BufferAttribute(dexterousColors, 3));

                    const dexterousMaterial = new THREE.PointsMaterial({
                        size: dexterityData.length > 100000 ? 2 : 3,
                        vertexColors: true,
                        transparent: true,
                        opacity: 0.9
                    });

                    const dexterousCloud = new THREE.Points(dexterousGeometry, dexterousMaterial);
                    scene.add(dexterousCloud);
                    workspacePoints.push(dexterousCloud);
                }

                // 计算体积
                const reachableVolume = computeWorkspaceVolume(reachablePoints, voxelSize);
                const dexterousPoints = dexterityData.filter(d => d.dexterity >= dexterousThreshold).map(d => d.point);
                const dexterousVolume = computeDexterous ? computeWorkspaceVolume(dexterousPoints, voxelSize) : 0;
                const percentage = reachableVolume > 0 ? (dexterousVolume / reachableVolume * 100) : 0;

                // 统计灵活度分布
                const avgDexterity = dexterityData.length > 0 
                    ? dexterityData.reduce((sum, d) => sum + d.dexterity, 0) / dexterityData.length 
                    : 0;

                // 更新UI
                if (reachableVolumeVal) {
                    reachableVolumeVal.textContent = `${reachableVolume.toLocaleString(undefined, { maximumFractionDigits: 6, minimumFractionDigits: 3 })}`;
                }
                
                if (computeDexterous && dexterousVolumeVal && dexterousPercentageVal) {
                    dexterousVolumeVal.textContent = `${dexterousVolume.toLocaleString(undefined, { maximumFractionDigits: 6, minimumFractionDigits: 3 })}`;
                    dexterousPercentageVal.textContent = `${percentage.toFixed(2)}`;
                }
                
                progressText.textContent = computeDexterous 
                    ? `完成 (可达:${reachablePoints.length.toLocaleString()}点, 灵活:${dexterousPoints.length}点, 平均灵活度:${avgDexterity.toFixed(3)})`
                    : `完成 (${reachablePoints.length.toLocaleString()}点)`;
            }
        } finally {
            isRendering = false;
            btn.textContent = '🔮 渲染工作空间';
            btn.onclick = renderWorkspace;

            setTimeout(() => {
                progressContainer.style.display = 'none';
            }, 3000);
        }
    }

    function clearWorkspace() {
        // 清除所有工作空间点云
        workspacePoints.forEach(pointCloud => {
            scene.remove(pointCloud);
            pointCloud.geometry.dispose();
            pointCloud.material.dispose();
        });
        workspacePoints = [];
        
        if (isRendering) {
            cancelRendering = true;
        }

        // 重置体积显示
        const volumeBox = document.getElementById('workspace-volume');
        const reachableVolumeVal = document.getElementById('reachable-volume-value');
        const dexterousInfo = document.getElementById('dexterous-info');
        const dexterousVolumeVal = document.getElementById('dexterous-volume-value');
        const dexterousPercentageVal = document.getElementById('dexterous-percentage-value');
        
        if (volumeBox) {
            volumeBox.style.display = 'none';
        }
        if (reachableVolumeVal) {
            reachableVolumeVal.textContent = '--';
        }
        if (dexterousInfo) {
            dexterousInfo.style.display = 'none';
        }
        if (dexterousVolumeVal) {
            dexterousVolumeVal.textContent = '--';
        }
        if (dexterousPercentageVal) {
            dexterousPercentageVal.textContent = '--';
        }
    }

    /**
     * 保存当前视野的超高清截图
     * 通过提高渲染分辨率实现高质量输出
     */
    function takeScreenshot() {
        if (!renderer || !scene || !camera) return;

        const container = document.getElementById('canvas-container');
        const width = container.clientWidth;
        const height = container.clientHeight;

        // 记录当前设置
        const oldPixelRatio = renderer.getPixelRatio();
        const oldSize = new THREE.Vector2();
        renderer.getSize(oldSize);

        // 提升分辨率
        renderer.setPixelRatio(oldPixelRatio * SCREENSHOT_SCALE);
        renderer.setSize(width, height, false);

        // 渲染一次高分辨率画面
        renderer.render(scene, camera);

        // 导出图片
        const dataURL = renderer.domElement.toDataURL('image/png');
        const link = document.createElement('a');
        const ts = new Date().toISOString().replace(/[:.]/g, '-');
        link.download = `robot_arm_${ts}.png`;
        link.href = dataURL;
        link.click();

        // 恢复原设置
        renderer.setPixelRatio(oldPixelRatio);
        renderer.setSize(oldSize.x, oldSize.y, false);
    }

    // ========================================
    // 配置保存/加载
    // ========================================

    function saveConfig() {
        const angleUnitSetting = document.getElementById('angle-unit').value || 'degree';

        const config = {
            version: '2.0',
            settings: {
                linkRadius: parseFloat(document.getElementById('link-radius').value),
                axisSize: parseFloat(document.getElementById('axis-size').value),
                jointDiameter: parseFloat(document.getElementById('joint-diameter').value),
                jointHeight: parseFloat(document.getElementById('joint-height').value),
                angleUnit: document.getElementById('angle-unit').value,
                showAxes: document.getElementById('show-axes').checked,
                showLabels: document.getElementById('show-labels').checked
            },
            joints: joints.map((j, idx) => ({
                type: j.type,
                x: j.x,
                y: j.y,
                z: j.z,
                rx: j.rx,
                ry: j.ry,
                rz: j.rz,
                min: j.min !== undefined ? j.min : (j.type === 'P' ? -500 : -180),
                max: j.max !== undefined ? j.max : (j.type === 'P' ? 500 : 180),
                visible: j.visible !== undefined ? j.visible : true,
                linkMode: j.linkMode || 'auto',
                value: (() => {
                    const raw = jointValues[idx] !== undefined ? jointValues[idx] : 0;
                    if (j.type === 'R') {
                        // 保存时按当前角度单位写入，便于人读
                        return angleUnitSetting === 'degree' ? raw * 180 / Math.PI : raw;
                    }
                    return raw; // 平动关节直接保存 mm
                })(),
                valueUnit: (() => {
                    if (j.type === 'R') {
                        return angleUnitSetting === 'degree' ? 'degree' : 'radian';
                    }
                    return 'mm';
                })()
            }))
        };

        const json = JSON.stringify(config, null, 2);
        const blob = new Blob([json], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = 'robot_arm_config.json';
        a.click();
        URL.revokeObjectURL(url);
    }

    function loadConfig(event) {
        const file = event.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = function(e) {
            try {
                const config = JSON.parse(e.target.result);

                if (config.settings) {
                    document.getElementById('link-radius').value = config.settings.linkRadius || 25;
                    document.getElementById('axis-size').value = config.settings.axisSize || 3;
                    document.getElementById('joint-diameter').value = config.settings.jointDiameter || 60;
                    document.getElementById('joint-height').value = config.settings.jointHeight || 60;
                    document.getElementById('angle-unit').value = config.settings.angleUnit || 'degree';
                    document.getElementById('show-axes').checked = config.settings.showAxes !== false;
                    document.getElementById('show-labels').checked = config.settings.showLabels !== false;
                }

                joints = [];
                jointValues = [];
                jointIdCounter = 0;

                if (config.joints) {
                    config.joints.forEach(j => {
                        joints.push({
                            id: jointIdCounter++,
                            type: j.type || 'R',
                            x: j.x !== undefined ? j.x : 0,
                            y: j.y !== undefined ? j.y : 0,
                            z: j.z !== undefined ? j.z : 100,
                            rx: j.rx !== undefined ? j.rx : 0,
                            ry: j.ry !== undefined ? j.ry : 0,
                            rz: j.rz !== undefined ? j.rz : 0,
                            min: j.min !== undefined ? j.min : (j.type === 'P' ? -500 : -180),
                            max: j.max !== undefined ? j.max : (j.type === 'P' ? 500 : 180),
                            visible: j.visible !== undefined ? j.visible : true,
                            linkMode: j.linkMode || 'auto'
                        });
                        const rawVal = j.value !== undefined ? parseFloat(j.value) : 0;
                        if (j.type === 'R') {
                            const unit = j.valueUnit || 'radian';
                            const val = unit === 'degree' ? rawVal * Math.PI / 180 : rawVal;
                            jointValues.push(isFinite(val) ? val : 0);
                        } else {
                            jointValues.push(isFinite(rawVal) ? rawVal : 0);
                        }
                    });
                }

                renderJointTable();
                updateVisualization();
                updateControlPanel();

                alert('配置加载成功！');
            } catch (err) {
                alert('配置文件格式错误: ' + err.message);
            }
        };
        reader.readAsText(file);
        event.target.value = '';
    }

    // ========================================
    // 工具函数
    // ========================================

    function debounce(fn, delay) {
        let timer = null;
        return function(...args) {
            clearTimeout(timer);
            timer = setTimeout(() => fn.apply(this, args), delay);
        };
    }

    // ========================================
    // 公开API
    // ========================================

    return {
        init,
        addJoint,
        deleteJoint,
        updateJointParam,
        updateJointType,
        updateJointVisibility,
        updateJointLinkMode,
        toggleSidebar,
        exportMDH,
        resetJoints,
        onSliderChange,
        renderWorkspace,
        clearWorkspace,
        saveConfig,
        loadConfig,
        toggleIK,
        toggleInstructions,
        takeScreenshot
    };

})();

// 页面加载完成后初始化
document.addEventListener('DOMContentLoaded', RobotArm.init);

