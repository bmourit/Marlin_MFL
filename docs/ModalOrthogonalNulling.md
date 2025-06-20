# Modal-Orthogonal Nulling (MON)  
*A Modal-Projection-Based Feedforward Cancellation Technique for 3D Printer Resonance*

**Author**: B. Mouritsen  
**Firmware**: Marlin with Fixed-Time Motion (FTM)  
**License**: GNU GPL v3+  

---

## Abstract

This document introduces **Modal-Orthogonal Nulling (MON)**, a mathematically precise feedforward compensation technique for suppressing mechanical resonance in 3D printers. MON computes a corrective kernel that is orthogonal to a target resonant mode, ensuring zero modal excitation. Unlike filtering or feedback approaches, MON operates purely in the trajectory domain and integrates directly into fixed-time motion controllers like Marlin's FTM.

---

## Motivation

3D printers suffer from artifacts such as **ghosting** or **ringing** due to mechanical resonances in their motion systems. Traditional solutions include:

- Input shaping (ZV, EI, 2HUMP)
- Notch filtering
- B-spline feedforward (e.g. Ulendo)
- Closed-loop damping with accelerometers

Each has tradeoffs in delay, smoothness, or hardware cost. **MON** instead uses **analytical orthogonality** to generate a trajectory-correction kernel that **precisely nullifies projection onto resonant modes**.

---

## Theoretical Foundations

### 1. Modal Projection

Given a resonant mode defined by frequency `f₀`, we model it as:

```
sin(ω₀t) where ω₀ = 2πf₀
```

The projection of a velocity profile `v(t)` onto this mode is:

```
P = ∫₀ᵀ v(t) * sin(ω₀t) dt
```

If this integral is non-zero, motion will excite the mode. **Our goal is to cancel this projection** by adding a correction function `k(t)` such that:

```
∫₀ᵀ [v(t) + k(t)] * sin(ω₀t) dt = 0
```

---

### 2. Nulling Kernel Form

We define the correction kernel `k(t)` as:

```
k(t) = A * sin(ω₀t + φ) * exp(-βt)
```

Where:

- `A` = amplitude  
- `φ` = phase  
- `β` = exponential decay rate (from physical damping)

The condition for orthogonality becomes:

```
∫₀ᵀ k(t) * sin(ω₀t) dt = -P
```

---

### 3. Solving for A and φ

Using trigonometric identities, we define two key integrals:

- `I₁ = ∫₀ᵀ sin²(ω₀t) * exp(-βt) dt`  
- `I₂ = ∫₀ᵀ sin(ω₀t) * cos(ω₀t) * exp(-βt) dt`

Then the condition becomes:

```
A * [I₁ * cos(φ) + I₂ * sin(φ)] = -P
```

We solve for optimal `φ` using:

```
φ = atan2(I₂, I₁)
```

And compute:

```
A = -P / (I₁ * cos(φ) + I₂ * sin(φ))
```

---

## Implementation in Marlin

### Step 1: Segment Analysis

Each motion segment is analyzed for:

- `start_speed`, `peak_speed`, `end_speed`  
- `acceleration`, `segment_time`  
- time spent in accel, coast, decel

A modal projection `P` is computed from all three phases.

---

### Step 2: Kernel Construction

For each axis with a configured resonance frequency, a kernel `k(t)` is generated such that:

```
∫₀ᵀ k(t) * sin(ω₀t) dt = -P
```

The kernel is stored and reused for the duration of the segment.

---

### Step 3: Trajectory Correction

During trajectory generation (at fixed time intervals), the kernel is evaluated:

```cpp
correction = A * sin(ω₀t + φ) * exp(-βt)
```

An optional **position-space correction** is computed via analytical integration of the kernel to improve accuracy.

---

## Validation

Kernel effectiveness is measured by computing the residual projection:

```
R = ∫₀ᵀ [v(t) + k(t)] * sin(ω₀t) dt
```

If `|R| / |P| < 0.05`, the kernel is considered successfully nulling the mode.

---

## Advantages

- **Analytical accuracy**: No FFTs, estimators, or filters.  
- **Fast evaluation**: Suitable for MCUs.  
- **Modular**: Drop-in for Marlin or any fixed-time trajectory system.  
- **No delay**: Zero lookahead required.  
- **Hardware-agnostic**: No sensors needed.  

---

## Limitations

- Requires known resonance frequency (manual input).  
- Less effective for low-amplitude resonance (may be hard to verify).  
- May need tuning of `β` for optimal decay alignment.  

---

## Applications Beyond Marlin

While implemented in Marlin’s FTM engine, this technique can generalize to:

- Any motion planner with segment-level timing info  
- CNC machines and pick-and-place heads  
- Feedforward vibration control in robotics  

---

## Source Code

This technique is implemented in the Marlin firmware under:

```
/src/module/modal_nulling.cpp
```

Enable with:

```cpp
#define MODAL_NULLING
```

And configure parameters in:

```
Configuration_adv.h
```

---

## Acknowledgements

This technique was developed independently as an improvement to prior feedforward shaping methods such as:

- Ulendo’s Filtered B-Splines  
- Input Shaping (ZV, EI)  
- Classical prefiltering

It is inspired by the principle of minimizing modal energy injection, rather than output tracking.

---

## License

This document and the associated code are licensed under the GNU General Public License v3 or later.
