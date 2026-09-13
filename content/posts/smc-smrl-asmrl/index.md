---
title: "从滑模控制到自适应滑模趋近律：理解一阶 SMC、SMRL 与分区趋近思想"
date: 2026-09-13
math: true
tags: ["滑模控制", "滑模趋近律", "鲁棒控制", "SMC", "ASMRL"]
keywords: ["滑模控制", "SMC", "滑模趋近律", "SMRL", "自适应滑模趋近律", "ASMRL", "抖振", "chattering", "PMSM"]
summary: "用直观方式介绍一阶滑模控制（SMC）、滑模趋近律（SMRL）与自适应滑模趋近律（ASMRL），核心围绕一个矛盾：系统距离滑模面远近不同时，是否应采用不同的趋近策略。"
---


> 本文希望用尽量直观的方式介绍一阶滑模控制（Sliding Mode Control, SMC）的基本思想，并进一步说明滑模趋近律（Sliding Mode Reaching Law, SMRL）以及自适应滑模趋近律（Adaptive Sliding Mode Reaching Law, ASMRL）的设计逻辑。重点不是复杂公式推导，而是理解一个核心问题：**系统距离滑模面很远和很近时，是否应该采用相同的趋近策略？**

---

## 1. 什么是滑模控制？

滑模控制（SMC）是一类典型的非线性鲁棒控制方法，其思想源头可追溯到苏联学者在 1950 年代开创的**变结构控制（Variable Structure Control, VSC）** [1]。需要说明的是，变结构控制与滑模控制并不完全等同：变结构控制强调的是“结构切换”——控制律不是时间的连续函数，而是随系统状态所处区域的不同，在若干连续控制律之间（可能以极高频率）切换；滑模控制则是变结构控制中最重要的一种工作模式，它进一步要求系统状态被“抓住”在预先设计的切换面上并沿其滑动，形成**滑动模态（sliding mode）**，从而获得对参数摄动和匹配扰动的强鲁棒性 [2]。换言之，变结构控制提供了“切换”这一设计框架，滑模控制则规定了切换后的具体运动形态——滑动模态。

值得指出的是，苏联学者最初研究变结构控制的动机之一，是把它应用于**导弹制导**等强非线性、强不确定性系统。在导弹拦截飞行目标的问题中，制导律本质上就是一个非线性控制律——需要让导弹在剧烈气动参数变化、目标机动、外部扰动等条件下，依然保持视线角速率稳定收敛到零。变结构控制所具备的对参数摄动和匹配扰动的不变性，恰好契合了这类场景对鲁棒性的极高要求。下图给出导弹拦截目标的制导几何示意：

<figure>
  <img src="/images/smc-smrl-asmrl/fig1-soviet-missile-guidance.png?v=2"
       alt="导弹拦截目标的制导几何示意：导弹沿弧形轨迹飞向飞行目标，二者之间用虚线表示视线"
       loading="lazy" />
  <figcaption><strong>图 1</strong>　导弹拦截目标的制导几何示意：导弹沿弧形轨迹飞向飞行目标，二者之间用虚线表示视线（LOS）。变结构控制所具有的鲁棒性，恰好契合这类强不确定性场景。</figcaption>
</figure>

它的基本思想并不复杂：首先人为设计一个能够代表系统期望动态的**滑模面（sliding surface）**，然后设计控制器，使系统状态在有限时间内到达该滑模面，并沿滑模面运动到最终平衡点。

因此，一个典型的滑模控制过程可以划分为两个阶段：

1. **趋近阶段（reaching phase）**：系统状态从初始位置向滑模面运动；
2. **滑模阶段（sliding phase）**：状态到达滑模面以后，沿滑模面继续运动并收敛到目标状态。

考虑一个简单的一阶非线性系统

$$
\dot{x}=f(x)+bu+d(t)
$$

其中 $x$ 为系统状态，$u$ 为控制输入，$f(x)$ 表示系统已知或可建模的动态，$d(t)$ 表示外部扰动或模型不确定性，$b$ 为控制增益。

如果控制目标是使 $x$ 跟踪参考值 $x^\ast$，可以定义误差 $e=x-x^\ast$，并在最简单的情况下选择滑模变量 $s=e$，控制目标于是可以转化为 $s\rightarrow 0$。对于更高阶系统，也经常采用 $s=\dot e+\lambda e$ 等形式构造滑模面。从这个角度看，滑模控制的核心实际上是：

$$
\boxed{\text{设计 } s \quad+\quad \text{设计系统如何趋近 } s=0}
$$

---

## 2. 为什么滑模控制具有较强的鲁棒性？

滑模控制最有吸引力的特点之一，是它对模型不确定性和外部扰动具有较好的鲁棒性。当系统进入理想滑模状态 $s=0$ 后，系统运动主要由所设计的滑模面决定，对于满足匹配条件的参数变化和扰动，其影响能够在一定程度上被切换控制作用抑制。

因此，SMC 被广泛应用于：

- 电机驱动与伺服系统；
- 功率电子变换器；
- 机器人控制；
- 飞行器与车辆控制；
- 非线性系统；
- 存在参数摄动和外部扰动的系统。

特别是在电机控制中，负载扰动、参数变化以及模型不确定性非常常见，因此 SMC 一直是一种重要的速度和位置控制方法。

---

## 3. SMC 的主要优点与问题

### 3.1 主要优点

SMC 的优势可以概括为：

- 对匹配扰动和参数不确定性具有较强鲁棒性；
- 适合非线性系统；
- 动态响应较快；
- 控制结构相对清晰；
- 不需要依赖极高精度的系统模型。

但是，经典 SMC 也存在一个非常著名的问题：

$$
\boxed{\text{Chattering}}
$$

也就是**抖振**。传统 SMC 中经常包含类似 $\operatorname{sgn}(s)$ 的非连续切换项。理论上，控制器希望在 $s=0$ 两侧进行无限快切换；但实际系统受到采样频率、开关频率、计算延迟、执行器带宽等限制，不可能实现无限频率切换。因此，系统通常会在滑模面附近不断来回运动，产生高频抖振。

---

## 4. 一个更重要的矛盾：速度与抖振

抖振问题背后其实存在一个更本质的矛盾：

> **为了让系统更快地到达滑模面，我们通常希望增加趋近控制强度；但过大的控制强度在滑模面附近又容易加剧抖振。**

也就是说，$\text{Large gain}\Rightarrow\text{Fast reaching}$，但同时 $\text{Large gain}\Rightarrow\text{More chattering near }s=0$。反过来，如果减小控制增益，$\text{Small gain}\Rightarrow\text{Smooth response}$，但又可能导致 $\text{Small gain}\Rightarrow\text{Slow reaching}$。

因此，传统滑模控制中一个非常重要的问题就是：

$$
\boxed{\text{Reaching speed vs. chattering}}
$$

后面的 SMRL 和 ASMRL，本质上都可以从这个问题出发理解。

---

## 5. 从滑模条件到滑模趋近律

传统滑模控制通常要求满足趋近条件，例如 $s\dot{s}<0$。它说明：只要系统状态位于滑模面的一侧，其运动方向应该指向滑模面。

但是，这个条件主要回答：

> **系统会不会向滑模面运动？**

它并没有直接回答：

> **系统应该以怎样的速度向滑模面运动？**

Gao 和 Hung 在经典工作中提出了 **reaching law method** [3]，即不只是判断 $s\dot{s}<0$，而是进一步直接规定滑模变量 $s$ 的动态，其基本思想可以写成

$$
\dot{s}=F(s).
$$

这样一来，我们可以直接设计系统在趋近阶段的动态性能。一个非常典型的指数趋近律为

$$
\boxed{\dot{s}=-ks-\varepsilon\operatorname{sgn}(s)}
$$

其中 $k>0,\ \varepsilon>0$。这个公式虽然简单，却很好地体现了滑模趋近律的思想。其中 $-ks$ 可以看作与距离滑模面相关的连续趋近项，而 $-\varepsilon\operatorname{sgn}(s)$ 则提供了指向滑模面的切换作用。

---

## 6. 基于 SMRL 的简单一阶滑模控制设计

仍然考虑系统 $\dot{x}=f(x)+bu+d(t)$，并定义 $s=x-x^\ast$，于是

$$
\dot{s} = f(x)+bu+d(t)-\dot{x}^\ast.
$$

现在人为规定期望的滑模趋近动态 $\dot{s}=-ks-\varepsilon\operatorname{sgn}(s)$。暂时忽略扰动项，或者假设切换项具有足够的鲁棒裕度，则可以得到控制输入

$$
u=\frac{1}{b}\left[-f(x)+\dot{x}^\ast-ks-\varepsilon\operatorname{sgn}(s)\right].
$$

这个设计过程实际上非常重要：

$$
\boxed{\text{先设计期望的 }\dot{s}\quad\Longrightarrow\quad\text{再根据系统模型反推出 }u}
$$

这就是基于 reaching law 设计滑模控制器最直观的理解方式之一。

---

## 7. 系统需要多久到达滑模面？

趋近律的另一个优点，是它可以直接用于分析系统的**趋近时间（reaching time）**。考虑 $\dot{s}=-ks-\varepsilon\operatorname{sgn}(s)$，假设初始时刻 $s(0)=s_0>0$，那么有 $\dot{s}=-ks-\varepsilon$，其解可以写为

$$
s(t) = \left(s_0+\frac{\varepsilon}{k}\right)e^{-kt}-\frac{\varepsilon}{k}.
$$

当系统第一次到达滑模面时 $s(t_r)=0$，因此可以得到

$$
\boxed{t_r=\frac{1}{k}\ln\left(1+\frac{k|s_0|}{\varepsilon}\right)}
$$

这个公式的物理意义是：

- $k$ 增大，通常可以提高趋近速度；
- $\varepsilon$ 增大，也可以增强系统向滑模面的驱动力；
- 初始状态距离滑模面越远，所需趋近时间通常越长。

这也是为什么**趋近律参数会直接决定滑模控制器的动态性能**。

---

## 8. 一个简单的 Lyapunov 稳定性证明

滑模控制中通常采用 Lyapunov 方法证明稳定性。选择 $V=\frac{1}{2}s^2$，显然 $V\geq0$，对其求导可得 $\dot V=s\dot{s}$。代入指数趋近律 $\dot{s}=-ks-\varepsilon\operatorname{sgn}(s)$，可得

$$
\dot V = -ks^2-\varepsilon |s|.
$$

对于 $s\neq0$，有 $\dot V<0$，因此系统状态会不断向 $s=0$ 趋近。

这里可以把两个分析方法区分开：

> **Lyapunov 稳定性分析主要回答“系统会不会稳定地趋近滑模面”；趋近时间分析则进一步回答“系统需要多长时间到达滑模面”。**

---

## 9. 固定趋近律的问题在哪里？

到这里，一个问题自然出现了。经典趋近律 $\dot{s}=-ks-\varepsilon\operatorname{sgn}(s)$ 中的参数 $k$ 和 $\varepsilon$ 通常是固定的，但从控制需求来看，系统距离滑模面很远和很近时，其控制目标实际上并不完全相同。

当 $|s|\gg0$ 时，系统状态距离目标较远，这时候我们最关心的是：

$$
\boxed{\text{How fast can the system reach the sliding surface?}}
$$

因此希望有较强的趋近作用。但是，当 $|s|\rightarrow0$ 时，系统已经接近滑模面，此时我们更关心：

$$
\boxed{\text{How smoothly can the system enter and stay near the sliding surface?}}
$$

如果此时仍然保持很大的趋近增益，就容易产生较明显的抖振。因此可以得到一个非常直观的问题：

> **为什么整个趋近过程一定要使用完全相同的趋近策略？**

这正是分区趋近律和自适应趋近律设计的出发点。

---

## 10. Piecewise Reaching：把趋近过程划分成不同区域

一个自然的思想是根据系统当前距离滑模面的大小 $|s|$ 对趋近过程进行分区。例如，可以概念性地划分为：

$$
\begin{cases}
\text{Region I:} & |s|>\Delta_2\\[2mm]
\text{Region II:} & \Delta_1<|s|\leq\Delta_2\\[2mm]
\text{Region III:} & |s|\leq\Delta_1
\end{cases}
$$

这里并不是说所有分区滑模控制都必须采用三个区域，而是借此说明一种设计思想：

$$
\boxed{\text{Different regions should have different control objectives.}}
$$

---

### 10.1 Region I：远离滑模面——快速趋近

当 $|s|$ 较大时，说明系统误差较大，距离滑模面较远，此时首要目标是快速缩小误差，因此可以采用较强的趋近作用：

$$
\boxed{\text{Far from sliding surface}\Rightarrow\text{Fast reaching}}
$$

从控制参数上看，相当于给予较大的等效趋近增益。这一阶段不需要过度追求控制量的平滑性，因为此时系统最重要的任务是尽快进入滑模面附近。

---

### 10.2 Region II：过渡区域——逐步降低趋近强度

随着 $|s|$ 逐渐减小，系统开始接近滑模面，此时控制目标也应该逐渐从 *fast convergence* 过渡到 *smooth convergence*。因此，中间区域可以理解为一个**动态过渡区**，趋近强度不再保持最大值，而是随着 $|s|$ 的减小逐渐调整。

---

### 10.3 Region III：靠近滑模面——降低抖振

当 $|s|\approx0$ 时，系统已经非常接近期望状态，此时如果仍保持过强的切换作用，会让状态频繁穿越滑模面，从而加剧抖振。因此，此区域的控制目标变成：

$$
\boxed{\text{Low chattering}+\text{good steady-state performance}}
$$

也就是说：

$$
\boxed{\text{Near sliding surface}\Rightarrow\text{Smooth and conservative reaching}}
$$

---

## 11. 分区思想的核心：远处要快，近处要稳

如果只记住整篇文章中的一句话，可以是：

$$
\boxed{\text{远离滑模面时快速趋近，靠近滑模面时平稳趋近}}
$$

或者更直观地说：

> **远处要跑得快，近处要刹得稳。**

从滑模控制的角度看，这实际上是在解决传统固定趋近律中的一个根本矛盾：

$$
\boxed{\text{Fast reaching}\quad\text{vs.}\quad\text{Low chattering}}
$$

固定参数的趋近律通常只能在两者之间进行折中。而分区或者自适应趋近律则尝试让控制器根据当前状态自动改变趋近强度：

$$
\boxed{|s|\quad\Longrightarrow\quad\text{reaching strength}}
$$

---

## 12. 从 Piecewise Reaching 进一步到 ASMRL

如果把这种思想进一步推广，就可以构造自适应滑模趋近律（ASMRL）。其一般思想可以抽象为

$$
\dot{s} = -k(s)s-\varepsilon(s)\operatorname{sgn}(s)
$$

其中趋近参数不再完全是固定常数，而是与当前状态相关，例如 $k=k(|s|)$。于是：

- 当 $|s|$ 很大时，提高趋近能力；
- 当 $|s|$ 逐渐减小时，降低趋近强度；
- 当系统靠近 $s=0$ 时，避免不必要的剧烈切换。

概念上，也可以写成

$$
k(|s|) =
\begin{cases}
k_{\rm far}, & |s|>\Delta_2\\
k_{\rm trans}(|s|), & \Delta_1<|s|\leq\Delta_2\\
k_{\rm near}, & |s|\leq\Delta_1
\end{cases}
$$

并满足 $k_{\rm far}>k_{\rm near}$。

真正的 ASMRL 并不一定需要采用这种最简单的分段常数形式，实际研究中可以使用指数函数、连续非线性函数或者其他自适应函数，使不同区域之间的切换更加平滑。关键并不在于具体函数形式，而在于：

$$
\boxed{\text{让 reaching law 根据系统所处区域主动改变动态特性}}
$$

---

## 13. 论文实例：ASMRL 的分区趋近设计

以作者在 PMSM 速度控制研究中提出的 ASMRL 为例。它在经典趋近律的基础上加入了几项自适应机制，其中最核心的是一个被切换函数绝对值界定的指数函数项。其趋近律可以写成

$$
\begin{aligned}
\dot{s} = &-\xi\operatorname{sgn}(s)\left(\frac{E}{E+d}\right)(1+k_3|x_2|) - k_0 s\\
&- k_1|s|^a E^n \operatorname{sgn}(s) - k_2(e^{bE}-1)\operatorname{sgn}(s)\cdot X
\end{aligned}
$$

其中 $E=\sqrt{x_1^2+x_2^2}$ 表示状态空间中当前状态到平衡点的欧氏距离，$X$ 是一个分段开关：

$$
X = \begin{cases} 1, & \sigma_1<|s|<\sigma_1+\sigma_2\\[1mm] 0, & \text{otherwise} \end{cases}
$$

完成 Fig.1 分区的关键在最后一项 $-k_2(e^{bE}-1)\operatorname{sgn}(s)$。当状态远离滑模面（$E$ 较大）时，指数项 $e^{bE}-1$ 迅速增大，成为驱动状态趋近滑模面的主导项；当状态接近滑模面（$E$ 较小）时，指数项快速衰减，不再起主导作用。为了让这一项只在需要的地方发力，论文用四条平行于滑模面 $s=0$ 的状态面划出一个「加速区」，也就是 $X=1$ 的区域 $\sigma_1<|s|<\sigma_1+\sigma_2$。

这正是 Fig.1 所描绘的分区相轨迹：系统状态从 **initial states** $\{e(0),\dot e(0)\}$ 出发，先进入外侧的加速带（band），在指数项 $-k_2(e^{bE}-1)\operatorname{sgn}(s)$ 的驱动下快速趋近滑模面 $s = ce + \dot e = 0$；一旦进入加速带之间的过渡区，指数项退出，系统平稳收敛到原点。

<figure>
  <img src="/images/smc-smrl-asmrl/fig2-asmrl-reaching-law.png"
       alt="ASMRL 分区趋近律相轨迹图：−k₂(eᵇᴱ−1)sgn(s) 项作用下的 SMC 过程"
       loading="lazy" />
  <figcaption><strong>图 2</strong>　带 −k₂(eᵇᴱ−1)sgn(s) 项的 SMC 过程 [4]。红色相轨迹从上方的初始状态出发，进入加速区后快速趋近原点附近的滑模面 s = ce + ė = 0。</figcaption>
</figure>

也就是说，$|s|\uparrow\Rightarrow\text{strong reaching}$，而 $|s|\downarrow\Rightarrow\text{smooth reaching}$。这就是分区概念和自适应趋近律之间最直接的联系。

---

## 14. 为什么这种思想适合电机速度控制？

以 PMSM 速度控制为例。当速度指令突然变化或者发生较大的负载扰动时，速度误差可能迅速增大，对应的滑模变量也会增大，这时系统需要：

$$
\boxed{\text{strong dynamic response}}
$$

以尽快恢复目标转速。而当速度已经接近参考值以后，继续使用很大的切换增益并没有明显收益，反而可能造成转矩波动、电流波动、速度纹波以及高频控制动作。因此，从实际控制需求看：

$$
\boxed{\text{Large-error region: dynamics first}}
$$

而

$$
\boxed{\text{Small-error region: smoothness first}}
$$

ASMRL 正是将这种工程直觉转化为数学上的 reaching-law 设计。

---

## 15. 从传统 SMRL 到 ASMRL 的逻辑

整个发展过程可以用下面这条路线概括：

$$
\boxed{\text{SMC}\rightarrow\text{Reaching Condition}\rightarrow\text{SMRL}\rightarrow\text{Fixed-Gain Limitation}\rightarrow\text{Piecewise Reaching}\rightarrow\text{ASMRL}}
$$

其中：

### SMC

解决的问题是：

> 如何构造一个对扰动和参数变化具有较强鲁棒性的控制系统？

### SMRL

进一步解决：

> 系统应该按照怎样的动态规律到达滑模面？

### Piecewise / Adaptive SMRL

进一步思考：

> 系统距离滑模面很远和很近时，是否应该采用相同的趋近速度？

最终得到的设计原则非常简单：

$$
\boxed{\text{Far: Fast}\qquad\text{Near: Smooth}}
$$

---

## 16. 小结

滑模控制的核心并不仅仅是一个 $\operatorname{sgn}(s)$ 函数，真正重要的是如何设计系统从初始状态到滑模面的全过程。经典 reaching law 将这个问题明确地写成 $\dot{s}=F(s)$，使设计者能够主动塑造趋近动态。

但是，固定参数趋近律不可避免地需要在 *reaching speed* 与 *chattering suppression* 之间进行折中。分区趋近和 ASMRL 的思想则进一步提出：

> **控制器不需要在整个状态空间中采用完全相同的趋近策略。**

当系统距离滑模面较远时，提高趋近速度；当系统进入滑模面附近以后，降低趋近强度并改善平稳性。因此，ASMRL 的核心价值并不是简单地“增加一个复杂非线性函数”，而是：

$$
\boxed{\text{根据系统当前所处的趋近区域，主动塑造不同的滑模动态。}}
$$

这也为进一步研究更高性能的自适应滑模控制、有限时间滑模控制、高阶滑模控制以及电机驱动中的鲁棒控制提供了一个非常直观的出发点。

对滑模变结构控制更系统的理论学习与 MATLAB 仿真实现，可参阅教材 [5]；对变结构控制理论发展脉络的完整综述，可参阅文献 [6]。

---

## 参考文献

1. Emel'yanov SV. A method to obtain complex regulation laws using only the error signal or the regulated coordinate and its first derivatives. *Avtomat. i Telemekh.*, 18(10): 873–885, 1957.
2. Utkin VI. *Sliding Modes and Their Application in Variable Structure Systems*. Moscow: Mir Publishers, 1978.
3. Gao W, Hung JC. Variable structure control of nonlinear systems: A new approach. *IEEE Trans. Ind. Electron.*, 40(1): 45–55, 1993.
4. Zhang Z, Yang X, Wang W, Chen K, Cheung NC, Pan J. Enhanced Sliding Mode Control for PMSM Speed Drive Systems Using a Novel Adaptive Sliding Mode Reaching Law Based on Exponential Function. *IEEE Trans. Ind. Electron.*, 71(10): 11978–11988, 2024.
5. 刘金琨. *滑模变结构控制 MATLAB 仿真（第3版）：基本理论与设计方法*. 清华大学出版社, 2015.
6. Hung JY, Gao W, Hung JC. Variable structure control: A survey. *IEEE Trans. Ind. Electron.*, 40(1): 2–22, 1993.
