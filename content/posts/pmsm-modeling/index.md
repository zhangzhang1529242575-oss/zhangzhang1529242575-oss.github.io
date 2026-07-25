---
title: "永磁同步电机（PMSM）数学建模 — dq 同步旋转坐标系"
date: 2026-07-25
math: true
tags: ["PMSM", "数学建模", "坐标变换", "矢量控制", "dq模型"]
keywords: ["PMSM", "永磁同步电机", "dq模型", "Clarke变换", "Park变换", "矢量控制", "MTPA"]
summary: "聚焦 dq 同步旋转坐标系，系统推导 PMSM 的电压、磁链、转矩方程及状态空间表达式。区分 SPMSM 与 IPMSM 的建模差异，并讨论 MTPA 的基本原理。"
---

## 前言

永磁同步电机（Permanent Magnet Synchronous Motor, PMSM）具有高功率密度、高效率和宽调速范围等优势，已广泛应用于工业伺服、电动汽车、航空航天等领域。

从控制的角度看，三相自然坐标系（abc）下的 PMSM 模型具有**非线性、强耦合、时变参数**的特点，直接在 abc 坐标系中设计控制器极为困难。通过坐标变换，将模型映射到与转子同步旋转的 dq 坐标系中——交流量变为直流量，时变电感变为常数，交叉耦合项物理意义明确——是矢量控制体系的理论基石。

本文参照袁雷《现代永磁同步电机控制原理及MATLAB仿真》的建模框架，**侧重 dq 同步旋转坐标系**，依次推导电压方程、磁链方程、转矩方程和状态空间表达式，并讨论表贴式（SPMSM）与内置式（IPMSM）的建模差异。

## 基本假设

建立数学模型时采用以下常用假设（袁雷教材 §2.1）：

1. 三相定子绕组对称，轴线互差 $120^\circ$ 电角度；
2. 忽略铁心饱和效应，磁路线性；
3. 不计涡流和磁滞损耗；
4. 永磁体励磁磁场和气隙磁通正弦分布；
5. 忽略转子阻尼绕组；
6. 反电动势为正弦波形。

---

## 1. 坐标变换（概要）

从 abc 到 dq 需经过两步变换。此处给出结论，详细推导可参考袁雷教材 §2.2–§2.3。

### 1.1 Clarke 变换：abc → αβ

将三相静止坐标系变换到两相静止坐标系（α 轴与 a 轴重合，β 轴超前 90°），采用**幅值守恒**约束：

$$
\begin{bmatrix} f_\alpha \\ f_\beta \end{bmatrix}
= \frac{2}{3}
\begin{bmatrix}
1 & -\dfrac{1}{2} & -\dfrac{1}{2} \\[6pt]
0 & \dfrac{\sqrt{3}}{2} & -\dfrac{\sqrt{3}}{2}
\end{bmatrix}
\begin{bmatrix} f_a \\ f_b \\ f_c \end{bmatrix}
$$

### 1.2 Park 变换：αβ → dq

将静止 αβ 坐标系变换到与转子同步旋转的 dq 坐标系（d 轴对齐永磁体 N 极，q 轴超前 90°）：

$$
\begin{bmatrix} f_d \\ f_q \end{bmatrix}
=
\begin{bmatrix}
\cos\theta_e & \sin\theta_e \\
-\sin\theta_e & \cos\theta_e
\end{bmatrix}
\begin{bmatrix} f_\alpha \\ f_\beta \end{bmatrix}
$$

**变换的物理本质**：Clarke + Park 相当于将三相交流绕组等效为一套固定在转子上的正交直流绕组——从转子的视角看，定子电流、电压在稳态时均为直流量。

---

## 2. 自然坐标系（abc）模型（简略）

为保持推导完整性，简要列出 abc 坐标系下的基本方程，详细展开见袁雷教材 §2.1。

**电压方程向量形式**：

$$\boldsymbol{u}_{abc} = R_s \boldsymbol{i}_{abc} + \frac{d}{dt}\boldsymbol{\psi}_{abc}$$

**磁链方程**：

$$\boldsymbol{\psi}_{abc} = \boldsymbol{L}_{abc}(\theta_e) \boldsymbol{i}_{abc} + \boldsymbol{\psi}_{f,abc}(\theta_e)$$

电感矩阵 $\boldsymbol{L}_{abc}(\theta_e)$ 随转子位置 $\theta_e$ 变化（时变参数）且非对角（互感耦合），这是 abc 模型的根本困难所在。

---

## 3. dq 同步旋转坐标系模型（重点）

将 abc 下的电压、磁链方程经 Clarke + Park 变换，并利用旋转坐标系导数的关系：

$$\frac{d}{dt}\boldsymbol{f}_{dq} = \boldsymbol{T}_{\alpha\beta \to dq} \frac{d}{dt}\boldsymbol{f}_{\alpha\beta} + \omega_e \begin{bmatrix} 0 & 1 \\ -1 & 0 \end{bmatrix} \boldsymbol{f}_{dq}$$

得到 dq 坐标系下的完整数学模型。

### 3.1 电压方程

$$
\boxed{
\begin{cases}
u_d = R_s i_d + \dfrac{d\psi_d}{dt} - \omega_e \psi_q \\[10pt]
u_q = R_s i_q + \dfrac{d\psi_q}{dt} + \omega_e \psi_d
\end{cases}
}
$$

**逐项解读**：

| 项 | 物理含义 |
|---|---|
| $R_s i_d$, $R_s i_q$ | 定子电阻压降 |
| $d\psi_d/dt$, $d\psi_q/dt$ | 磁链变化产生的变压器电动势（暂态项） |
| $-\omega_e \psi_q$ | d 轴运动电动势——q 轴磁链在旋转中"切割"d 轴绕组产生 |
| $+\omega_e \psi_d$ | q 轴运动电动势——d 轴磁链在旋转中"切割"q 轴绕组产生 |

其中 $-\omega_e \psi_q$ 和 $+\omega_e \psi_d$ 是 dq 轴之间的**交叉耦合项**——它们使 d、q 轴不能完全独立控制，是矢量控制中需要前馈解耦的根本原因。

### 3.2 磁链方程

将 $\psi_d$、$\psi_q$ 用电流和永磁磁链显式表达：

$$
\boxed{
\begin{cases}
\psi_d = L_d i_d + \psi_f \\
\psi_q = L_q i_q
\end{cases}
}
$$

代入电压方程，得到以 $i_d$、$i_q$ 为状态变量的**电流动态方程**：

$$
\begin{cases}
u_d = R_s i_d + L_d \dfrac{di_d}{dt} - \omega_e L_q i_q \\[10pt]
u_q = R_s i_q + L_q \dfrac{di_q}{dt} + \omega_e (L_d i_d + \psi_f)
\end{cases}
$$

写成矩阵形式，便于控制器设计：

$$
\begin{bmatrix} u_d \\ u_q \end{bmatrix}
=
\begin{bmatrix} R_s + L_d s & -\omega_e L_q \\[4pt]
\omega_e L_d & R_s + L_q s
\end{bmatrix}
\begin{bmatrix} i_d \\ i_q \end{bmatrix}
+
\begin{bmatrix} 0 \\ \omega_e \psi_f \end{bmatrix}
$$

其中 $s = d/dt$ 为微分算子。

**关键观察**：
- 交叉耦合项 $-\omega_e L_q i_q$ 和 $+\omega_e L_d i_d$ 与转速成正比——**转速越高，耦合越强**；
- 永磁反电动势 $\omega_e \psi_f$ 只出现在 q 轴——与转速成正比，是电机发电效应的来源；
- 当 $\omega_e = 0$（静止），模型退化为两路独立的 RL 电路——d、q 轴完全解耦。

### 3.3 转矩方程

根据机电能量转换原理或功率平衡关系，dq 坐标系下的电磁转矩为：

$$
\boxed{
T_e = \frac{3}{2} n_p (\psi_d i_q - \psi_q i_d)
= \frac{3}{2} n_p \big[\psi_f i_q + (L_d - L_q)i_d i_q\big]
}
$$

**转矩的两分量分解**：

$$
T_e = \underbrace{\frac{3}{2} n_p \psi_f i_q}_{\text{永磁转矩 } T_{pm}} \;+\; \underbrace{\frac{3}{2} n_p (L_d - L_q) i_d i_q}_{\text{磁阻转矩 } T_{rel}}
$$

| 分量 | 表达式 | 来源 | 存在条件 |
|------|--------|------|----------|
| 永磁转矩 $T_{pm}$ | $\frac{3}{2}n_p \psi_f i_q$ | 永磁磁场 ↔ q 轴电流 | 始终存在 |
| 磁阻转矩 $T_{rel}$ | $\frac{3}{2}n_p (L_d - L_q)i_d i_q$ | dq 轴磁阻不对称（凸极效应） | $L_d \neq L_q$ 时存在 |

**磁阻转矩的符号**（以 $i_q > 0$ 电动状态为例）：
- 若 $L_d < L_q$（IPMSM 常见情况），$(L_d - L_q) < 0$，施加负向 $i_d$（$i_d < 0$）使 $i_d i_q < 0$ → 磁阻转矩为正，**贡献电动转矩**；
- 若 $L_d > L_q$（少数设计），则需正向 $i_d$ 利用磁阻转矩。

### 3.4 运动方程

$$
\frac{d\omega_m}{dt} = \frac{1}{J}(T_e - T_L - B\omega_m)
$$

$$
\frac{d\theta_e}{dt} = n_p \omega_m
$$

其中：$J$ 为转动惯量，$B$ 为粘滞摩擦系数，$T_L$ 为负载转矩，$\omega_m$ 为机械角速度（与电角速度关系：$\omega_e = n_p \omega_m$）。

---

## 4. 状态空间表达式

选取状态变量 $\boldsymbol{x} = [i_d, i_q, \omega_m]^T$，控制输入 $\boldsymbol{u} = [u_d, u_q]^T$，扰动输入 $T_L$，得到 PMSM 的**非线性状态空间模型**（袁雷教材 §2.4）：

### 电流动态子系统

$$
\boxed{
\begin{cases}
\dfrac{di_d}{dt} = -\dfrac{R_s}{L_d} i_d + \dfrac{L_q}{L_d} n_p \omega_m i_q + \dfrac{1}{L_d} u_d \\[14pt]
\dfrac{di_q}{dt} = -\dfrac{R_s}{L_q} i_q - \dfrac{L_d}{L_q} n_p \omega_m i_d - \dfrac{n_p \psi_f}{L_q} \omega_m + \dfrac{1}{L_q} u_q
\end{cases}
}
$$

### 转速动态子系统

$$
\frac{d\omega_m}{dt} = \frac{3n_p}{2J}\big[\psi_f i_q + (L_d - L_q)i_d i_q\big] - \frac{B}{J}\omega_m - \frac{1}{J}T_L
$$

### 非线性特征分析

状态方程中出现了三类非线性项：

1. **乘积项** $\omega_m i_d$、$\omega_m i_q$——转速 × 电流，属于**双线性（bilinear）非线性**；
2. **乘积项** $i_d i_q$——出现在转矩方程中（IPMSM），属于**二次非线性**；
3. **参数不确定性**——$R_s$ 随温度变化，$L_d$、$L_q$ 随磁饱和变化，$\psi_f$ 随温度变化。

这些非线性是 PMSM 高性能控制的难点所在——PI 控制器只在特定工作点附近有效，大范围调速和负载突变时性能下降。这自然地引出了后续的控制进阶话题：

- **前馈解耦**：抵消 $-\omega_e L_q i_q$ 和 $+\omega_e L_d i_d$ 交叉耦合项；
- **滑模控制（SMC）**：对参数摄动和外部扰动具有完全鲁棒性；
- **模型预测控制（MPC）**：显式利用非线性模型进行滚动优化；
- **自抗扰控制（ADRC）**：将非线性耦合视为"总扰动"进行估计和补偿。

---

## 5. SPMSM vs IPMSM

两者的本质区别在于转子永磁体的安装方式，导致 $L_d$ 与 $L_q$ 的关系不同。

| 特征 | SPMSM（表贴式） | IPMSM（内置式） |
|------|-----------------|-----------------|
| 永磁体位置 | 转子铁心表面 | 嵌入转子铁心内部 |
| dq 轴磁路 | 对称（永磁体 $\mu_r \approx 1$） | 不对称（q 轴磁路以硅钢为主，磁阻小） |
| 电感关系 | $L_d = L_q = L_s$ | 通常 $L_q > L_d$ |
| 凸极比 $\rho = L_q/L_d$ | $\rho = 1$ | $\rho > 1$（典型 1.5~3） |
| 转矩组成 | 仅永磁转矩 | 永磁转矩 + 磁阻转矩 |
| $i_d = 0$ 控制 | MTPA 等价于 $i_d = 0$ | $i_d = 0$ **不是** MTPA，需负向 $i_d$ |

### SPMSM：简化模型

$L_d = L_q = L_s$ 时，电压和转矩方程退化为：

$$
\begin{bmatrix} u_d \\ u_q \end{bmatrix}
=
\begin{bmatrix} R_s + L_s s & -\omega_e L_s \\[4pt]
\omega_e L_s & R_s + L_s s
\end{bmatrix}
\begin{bmatrix} i_d \\ i_q \end{bmatrix}
+
\begin{bmatrix} 0 \\ \omega_e \psi_f \end{bmatrix}
$$

$$
T_e = \frac{3}{2} n_p \psi_f i_q
$$

此时 $T_e \propto i_q$，控制极为简洁——这正是 SPMSM 在伺服系统中广泛应用的原因之一。

### IPMSM：MTPA 控制原理

IPMSM 的磁阻转矩为额外自由度。在一定 $T_e$ 需求下，寻找使电流幅值 $I_s = \sqrt{i_d^2 + i_q^2}$ 最小的 $(i_d, i_q)$ 组合，即为**最大转矩电流比（MTPA）**控制（袁雷教材 §3.3）。

由约束优化问题 $\min I_s^2 = i_d^2 + i_q^2$ s.t. $T_e = \frac{3}{2}n_p[\psi_f i_q + (L_d - L_q)i_d i_q]$，利用拉格朗日乘子法得到 MTPA 条件：

$$
\boxed{
i_d = -\frac{\psi_f}{2(L_d - L_q)} - \sqrt{\left[\frac{\psi_f}{2(L_d - L_q)}\right]^2 + i_q^2}
}
$$

（取负号是因为 IPMSM 中 $L_d < L_q$，$i_d$ 应为负值以产生正向磁阻转矩。）

或以电流矢量角 $\beta$ 描述（$i_d = -I_s \sin\beta$, $i_q = I_s \cos\beta$，$\beta$ 为电流超前 q 轴的角度）：

$$
T_e = \frac{3}{2}n_p\left[\psi_f I_s \cos\beta - \frac{1}{2}(L_d - L_q)I_s^2 \sin 2\beta\right]
$$

MTPA 角度 $\beta^*$ 可通过 $\partial T_e / \partial \beta = 0$ 求解。

---

## 6. 小结

本文聚焦 dq 同步旋转坐标系，建立了 PMSM 的完整数学模型。核心结论：

1. **坐标变换是桥梁**：Clarke + Park 将三相交流系统等效为 dq 直流系统，使参数定常、变量直流量化；
2. **dq 电压方程**：$u_d = R_s i_d + L_d \frac{di_d}{dt} - \omega_e L_q i_q$，$u_q = R_s i_q + L_q \frac{di_q}{dt} + \omega_e(L_d i_d + \psi_f)$——交叉耦合项 $-\omega_e L_q i_q$ 和 $+\omega_e L_d i_d$ 是前馈解耦的直接依据；
3. **转矩方程**：$T_e = \frac{3}{2}n_p[\psi_f i_q + (L_d - L_q)i_d i_q]$——SPMSM 仅永磁转矩（$\propto i_q$），IPMSM 含额外磁阻转矩（利用凸极性）；
4. **非线性特征**：乘积项 $\omega_m i_d$、$\omega_m i_q$、$i_d i_q$ 使 PMSM 为双线性非线性系统——这是研究滑模控制、MPC、ADRC 等先进控制方法的直接动机；
5. **SPMSM vs IPMSM**：前者模型简洁，$i_d=0$ 即 MTPA；后者需负向 $i_d$ 利用磁阻转矩，数学模型和控制策略均更复杂。

熟练掌握 dq 坐标系下的 PMSM 模型，是学习矢量控制（FOC）、弱磁控制、无传感器控制以及各类先进控制算法的基本前提。

## 参考文献

1. 袁雷, 胡冰新, 魏克银, 陈姝. *现代永磁同步电机控制原理及MATLAB仿真*. 北京航空航天大学出版社, 2016.
2. Pillay P, Krishnan R. Modeling, simulation, and analysis of permanent-magnet motor drives, Part I: The permanent-magnet synchronous motor drive. *IEEE Trans. Ind. Appl.*, 25(2): 265–273, 1989.
3. 王成元, 夏加宽, 孙宜标. *现代电机控制技术*. 机械工业出版社, 2014.
4. Krishnan R. *Permanent Magnet Synchronous and Brushless DC Motor Drives*. CRC Press, 2010.
