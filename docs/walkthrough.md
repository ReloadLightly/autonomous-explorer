# Trajectory Tracking Foundations: from wheels to evidence

This walkthrough follows the physical problem, equations, numerical method, code, observed behavior, and limits. Read alongside [Chapter 1](https://porabook.com/book/modeling-robot-dynamics/), [Chapter 2](https://porabook.com/book/open-loop-control/), and [Chapter 3](https://porabook.com/book/closed-loop-control/). The concrete exercise anchor is Chapter 3, Exercise 5. [Source revision and interface notes](sources.md) make that connection explicit.

## 1. The physical problem

We want a small wheeled rover to execute a timed path. Its wheels permit forward motion and turning, but it cannot translate sideways independently of its heading. A controller must therefore turn the rover to change the direction of motion. Figure [01](../media/01_robot_geometry.png) shows the body/world frames; Figure [02](../media/02_motion_primitives.png) shows commands becoming motion.

The drawing uses a 0.48 × 0.30 m body, two 0.22 × 0.07 m drive wheels on a 0.38 m track, a rear caster, and a pointed nose. These are illustrative dimensions used by one shared renderer. They do not imply a wheel-radius model, motor model, collision model, or physical robot. The same icon will carry forward into later milestones.

## 2. Equations and conventions

[`State`](../src/autonomous_explorer/model.py) names the state fields. Every numerical state row uses the order

$$s=(x,y,v,\theta), \qquad u=(a,\omega).$$

Position is in metres, speed in m/s, acceleration in m/s², heading in radians, and angular rate in rad/s. Heading is measured counterclockwise from world +x. The nominal model is

$$\dot x=v\cos\theta,\quad \dot y=v\sin\theta,\quad \dot v=a,\quad \dot\theta=\omega.$$

Multiplication by cosine and sine resolves forward speed along the two world axes. Acceleration integrates into stored forward speed, and yaw rate integrates into heading. This is a kinematic abstraction of a wheeled robot, extended with speed as a state. Forces, torques, slip, and actuator dynamics are outside the model. Chapter 1 motivates these modeling and numerical-integration foundations.

`derivative` exposes two exogenous offsets for the experiments:

$$\dot p=(v+d_v)[\cos\theta,\sin\theta]^T,\quad \dot v=a,\quad \dot\theta=\omega+d_\omega.$$

Here $d_v$ is a body-forward translation-speed offset [m/s], and $d_\omega$ is a yaw-rate offset [rad/s]. The first is **not** acceleration noise. Stored speed remains a distinct state even when instantaneous translation is disturbed. The seeded case can briefly reverse effective translational motion when $v+d_v<0$; this is an exercise-style perturbation, not a validated terrain model.

## 3. Numerical method: integrate held inputs

The simulator computes a command at $t_k$ and holds both command and the pre-generated disturbance sample throughout $[t_k,t_{k+1})$. [`step`](../src/autonomous_explorer/model.py) evaluates four derivative slopes:

$$k_1=f(s_k,u_k,d_k),$$
$$k_2=f(s_k+\tfrac h2 k_1,u_k,d_k),\quad
k_3=f(s_k+\tfrac h2 k_2,u_k,d_k),$$
$$k_4=f(s_k+h k_3,u_k,d_k),\qquad
s_{k+1}=s_k+\tfrac h6(k_1+2k_2+2k_3+k_4).$$

The midpoint slopes probe how orientation and speed change within a step; the weighted average improves on a single initial slope. Euler, retained only for comparison, uses $s_{k+1}=s_k+h f(s_k,u_k,d_k)$. No random number is drawn in either integrator. No controller is recomputed at an RK stage.

For a straight motion with constant acceleration, displacement is $(v_0t+at^2/2)[\cos\theta_0,\sin\theta_0]$. For constant speed $v$ and nonzero constant angular rate $\omega$,

$$\theta(t)=\theta_0+\omega t,$$
$$x(t)=x_0+\frac v\omega[\sin(\theta_0+\omega t)-\sin\theta_0],$$
$$y(t)=y_0-\frac v\omega[\cos(\theta_0+\omega t)-\cos\theta_0].$$

The separate $\omega=0$ branch uses the straight-line limit to avoid division by zero. Tests compare both motions to the integrator. Figure [03](../media/03_integration_accuracy.png) uses the saved constant-turn study to show the observed error order. Smaller timesteps reduce integration error; controller sample-and-hold error is a different source of error and need not converge at RK4's rate.

## 4. Construct the cubic reference

The inspected exercise prescribes $p_0=(0,0)$, $p_f=(5,5)$, $v_0=v_f=0.5$ m/s, $\theta_0=\theta_f=-\pi/2$, and $T=25$ s. Convert endpoint speed and heading to Cartesian velocities with $\dot p=v[\cos\theta,\sin\theta]^T$. Both endpoints therefore have velocity $(0,-0.5)$ m/s, up to floating-point roundoff.

[`CubicReference`](../src/autonomous_explorer/reference.py) uses normalized time $\tau=t/T$ for a well-scaled linear solve. For either coordinate,

$$p(\tau)=c_0+c_1\tau+c_2\tau^2+c_3\tau^3.$$

Rows of the following matrix enforce initial/final position and initial/final derivative with respect to $\tau$:

$$\begin{bmatrix}1&0&0&0\\1&1&1&1\\0&1&0&0\\0&1&2&3\end{bmatrix}
\begin{bmatrix}c_0\\c_1\\c_2\\c_3\end{bmatrix}
=\begin{bmatrix}p_0\\p_f\\T\dot p_0\\T\dot p_f\end{bmatrix}.$$

The two columns of `np.linalg.solve` solve x and y together. Multiplication by T on the right follows from $dp/d\tau=T\,dp/dt$. For these endpoints, ignoring roundoff,

$$x=15\tau^2-10\tau^3,\qquad
y=-12.5\tau+52.5\tau^2-35\tau^3.$$

Evaluate position with basis $[1,\tau,\tau^2,\tau^3]$, velocity with $[0,1,2\tau,3\tau^2]/T$, and acceleration with $[0,0,2,6\tau]/T^2$. The factors of T are required by the chain rule.

Recover reference speed with $v_d=\|\dot p_d\|$ and heading with $\theta_d=\operatorname{atan2}(\dot y_d,\dot x_d)$; `atan2` preserves the velocity quadrant. Signed curvature is $(\dot x_d\ddot y_d-\dot y_d\ddot x_d)/v_d^3$. Figure [04](../media/04_reference_construction.png) plots the saved reference arrays and these derivative quantities.

This is a feasible trajectory construction through **differential flatness**: position outputs and a finite number of their derivatives determine states and controls. It solves interpolation constraints. No minimum-time or minimum-energy objective is optimized. Chapter 2 supplies this prerequisite.

## 5. Map Cartesian acceleration into rover controls

Differentiate $\dot p=v[\cos\theta,\sin\theta]^T$ using the product and chain rules:

$$\ddot p=
\underbrace{\begin{bmatrix}\cos\theta&-v\sin\theta\\\sin\theta&v\cos\theta\end{bmatrix}}_{J(\theta,v)}
\begin{bmatrix}a\\\omega\end{bmatrix}.$$

The first column is the forward direction: acceleration changes speed along it. The second is the lateral direction multiplied by v: turning rotates the velocity vector. The determinant is $v(\cos^2\theta+\sin^2\theta)=v$. Project desired Cartesian acceleration $w$ onto these orthogonal directions:

$$a=\cos\theta\,w_x+\sin\theta\,w_y,$$
$$\omega=\frac{-\sin\theta\,w_x+\cos\theta\,w_y}{v}.$$

[`invert_acceleration`](../src/autonomous_explorer/control.py) implements these two projections. Tests multiply the resulting controls by J and recover w. The mapping fails at zero speed because turning a stationary velocity vector cannot create arbitrary lateral acceleration.

For open loop, evaluate the mapping using desired states and desired acceleration at every control timestamp **before execution**. [`OpenLoop.command(index)`](../src/autonomous_explorer/control.py) has no actual-state argument. A disturbance therefore cannot change this command sequence.

For feedback, [`Feedback.command(actual, desired)`](../src/autonomous_explorer/control.py) forms

$$w=\ddot p_d+K_p(p_d-p)+K_d(\dot p_d-v[\cos\theta,\sin\theta]^T).$$

The first term feeds forward the reference's acceleration. The second pushes position back toward the timed reference. The third damps Cartesian velocity error. $K_p=\operatorname{diag}(2,2)$ s⁻² and $K_d=\operatorname{diag}(2,2)$ s⁻¹ make all terms acceleration. These gains match the exercise's example gains; they were not fitted to the outcomes. In the ideal continuous nominal system, the position error obeys $\ddot e+K_d\dot e+K_pe=0$. Disturbances and sampled updates limit that ideal argument. Exact tracking makes both correction terms zero, so feedback equals feedforward; a test checks this identity.

Feedback receives exact **simulator state**, with velocity reconstructed from stored v and heading. It does not observe the exogenous samples or directly measure the extra instantaneous speed offset. There is no estimator or sensor model.

## 6. Guard the inversion domain

This implementation chooses the forward branch $v>10^{-4}$ m/s. It checks before each feedback inversion and at the final state. It raises `InvalidSpeed`, records the first invalid sampled timestamp, retains the valid history and offending sampled state, and reports no successful final error. There is no denominator replacement, clipping, or unreported regularization. Negative signed speeds also lie outside this chosen forward branch even though the algebraic inversion exists at negative nonzero v.

`minimum_speed` checks the **continuous reference**, not only its time samples: squared speed is a quartic polynomial in $\tau$, so all extrema occur at endpoints or real in-range roots of its cubic derivative. The minimum is 0.1373606 m/s, safely above the threshold. Actual speed is monitored separately. Within each held-input interval, stored v is affine in time; checking its sampled endpoints detects any threshold crossing, though the logged event time is the next sample rather than an interpolated crossing time.

The open-loop plant can evolve through zero speed because it never inverts actual v; its low-speed samples are still counted in metrics. The deliberately stopped feedback probe fails immediately and is saved separately from the six performance runs.

## 7. Read the execution evidence

[`paired_runs`](../src/autonomous_explorer/simulation.py) creates one disturbance array, then passes the same realization to both simulations. Initial state, reference, grid, and horizon are identical. Both use the same optional actuator limits; the release configuration sets them to null. Requested/applied controls and saturation flags are nevertheless retained.

The [experiment protocol](experiment.md) specifies all three cases and the log schema. Figures [05](../media/05_trajectory_comparison.png) and [06](../media/06_temporal_progression.png) connect full paths to time snapshots; the [diagnostic GIF](../media/diagnostic_replay.gif) connects those poses to signal histories. Figures [07–11](../MILESTONE_01.md#diagnose-the-behavior) expose errors, states, commands, delivered disturbances, and all declared outcomes.

Rendering consumes saved files and verifies their checksums. Metrics use those same arrays. Simulation and rendering are separate commands; tests replace simulation functions with failures while exercising the plotting route. The observed feedback advantage in the declared cases is evidence about this model and protocol, not a universal claim about feedback controllers.

## 8. What this experiment leaves open

This model assumes exact state, direct acceleration/yaw-rate actuation, flat terrain, and no actuator bounds. It does not cover standstill recovery, wheel slip physics, dynamic obstacles, collision avoidance, sensing delays, uncertain localization, or model mismatch beyond the specified offsets. A single seed cannot establish stochastic robustness. The endpoint remains moving. The next useful step is to give this same rover a Chapter 4 collision-aware planner, then introduce sensing and localization before attempting SLAM and exploration.
