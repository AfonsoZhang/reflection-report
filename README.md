# AAE5303 Robust Control Technology in Low-Altitude Aerial Vehicle

## Post-Lesson Reflection Report

**Student Name:** ZHANG Shuyang  
**Student ID:** 25049993G  
**Date:** [2026/4/24]

---

## Section 1: AI Usage Experience

I used AI coding assistants throughout the entire course, with the tools and usage patterns evolving as the tasks grew more complex.

In **Assignment 1** (environment setup), I primarily used Poe to troubleshoot Docker and conda issues — for example, resolving a Python version incompatibility with Open3D (required 3.11, had 3.13) and accepting Anaconda's Terms of Service before creating environments. Poe was convenient for quick, targeted Q&A when I needed immediate answers about command-line errors.

In **Assignment 2** (HKisland VO), I escalated to using Cursor more actively for C++ modifications to ORB-SLAM3 source code — modifying `Tracking.cc` to lower the inlier threshold from 30 to 20 for sparse aerial scenes, and `System.cc` to enable monocular trajectory export. AI also helped write the evaluation pipeline with `evo`.

In the **final project** (AMtown02), I relied on Cursor's Agent mode almost daily. AI autonomously executed multi-step workflows: creating six YAML configs for the k1 ablation experiment, running all evaluations, generating matplotlib plots, and managing 99 Git commits. The features I relied on most were: (1) Agent mode for autonomous multi-step execution, (2) terminal integration for running `evo_ape` and `mono_euroc` directly, and (3) codebase-aware context that let the AI reference existing config files.

---

## Section 2: Understanding AI Limitations

The most significant AI failure occurred during my calibration analysis in the final project. When I was documenting why using HKisland's configuration on AMtown02 data produced an ATE of 215 meters, the AI fabricated a distortion coefficient value of `k1 = -0.189` and attributed the high ATE primarily to this k1 difference. This value **does not exist** in any calibration file in the repository.

The actual root cause was fundamentally different: the 215m ATE was caused by using mismatched **intrinsic parameters** (fx, fy, cx, cy) from a different dataset's camera. Using the wrong projection model is a catastrophic error regardless of distortion.

I detected this hallucination by manually cross-referencing the calibration files (`calib_yaml/HK_GNSS(airport & island).yaml` and `calib_yaml/HKisland.yaml`). The value `k1 = -0.189` appeared in neither file. I corrected the AI explicitly, and it revised the entire `CALIBRATION_ANALYSIS.md`. This taught me that AI can construct convincing but fabricated causal narratives around numerical data — a particularly dangerous failure mode in engineering.

---

## Section 3: Engineering Validation

Across all assignments, I applied progressively more rigorous validation:

**Assignment 2 — baseline comparison.** I compared my tuned ORB-SLAM3 (ATE 1.72m) against the course-provided baseline (ATE 88.2m) on HKisland_GNSS03, confirming that my modifications (tracking thresholds, feature parameters, 0.5× downsampling) achieved a 98% improvement.

**Final project — controlled ablation.** To verify k1's impact on AMtown02, I designed a rigorous ablation study: varying only k1 across six values while holding all other parameters constant. The results confirmed a 46× ATE variation (2.39m to 110.7m), providing causal rather than correlational evidence.

**Final project — dual ground truth cross-validation.** I evaluated the best trajectory against two independent sources: SfM reconstruction (ATE 2.310m) and RTK GPS (ATE 2.647m). The consistency confirmed trajectory quality was genuine, not an artifact of one particular reference.

**Final project — pipeline validation on TUM-VI.** After VIO failed on AMtown02, I ran it on TUM-VI room1 (ATE 0.011m, scale 0.999), confirming the failure was dataset-specific, not an implementation bug.

---

## Section 4: Problem-Solving Process

The most challenging problem was understanding why Mono-Inertial SLAM completely failed on the AMtown02 dataset despite having 400 Hz IMU data available.

**Initial symptom:** Every Mono-Inertial run produced "Fail to track local map!" errors within seconds, regardless of IMU noise parameters or initialization settings.

**Diagnosis:** I first tested multiple IMU noise configurations — no improvement. I then tried all four rotation permutations for camera-IMU extrinsic `T_b_c1` — all failed. The AI suggested checking gimbal angle data from `/dji_osdk_ros/gimbal_angle`.

**Root cause:** By plotting gimbal yaw over time, I discovered it changed by over 100° between survey legs. ORB-SLAM3 assumes a **constant** `T_b_c1`, but the 3-axis gimbal makes this transformation **time-varying** — a fundamental architectural incompatibility.

**Solution attempt:** I developed a Virtual IMU in C++ (`ros_mono_inertial_virtual_imu.cc`), transforming body-frame IMU readings into the camera frame using gimbal angle interpolation: `a_cam(t) = R_cb(t) · a_body(t)`. This required solving Eigen SIMD alignment crashes and handling numerical differentiation noise from 50 Hz gimbal data.

**Outcome:** The Virtual IMU was physically correct (|a| ≈ 9.81 verified), but VIO still failed — at high altitude, the downward-looking camera lacks parallax for scale estimation. TUM-VI validation (ATE 0.011m) proved the algorithm works on suitable data.

**AI's role:** AI efficiently implemented the C++ code and debugged Eigen crashes, but could not independently identify the gimbal as root cause — that insight came from my own analysis.

---

## Section 5: Learning Growth

At the start of the course, Docker and ROS felt impenetrable — Assignment 1's environment setup was my first encounter with containers, conda environments, and ROS 2 workspaces, and I relied heavily on Poe to understand error messages. By Assignment 2, I could modify ORB-SLAM3 C++ source code (`Tracking.cc`, `System.cc`), tune parameters, and run evaluations independently. By the final project, I was writing new ROS nodes from scratch and designing controlled experiments.

My most significant growth was understanding **how calibration propagates through a SLAM pipeline**: incorrect distortion → wrong feature undistortion → corrupted matching → trajectory drift. The k1 ablation study made this visceral — watching ATE jump from 2.4m to 110.7m by changing one number. I also developed skills in experimental methodology — controlled ablation, multi-run statistics, and cross-validation — that I previously only knew from reading papers.

---

## Section 6: Critical Reflection

AI was overwhelmingly positive for productivity across all three assignments. In Assignment 1, Poe helped me resolve conda and Docker issues in minutes rather than hours. Assignment 2 benefited from Cursor's AI-assisted C++ modifications. The final project's 99 commits and systematic experiments would have been impractical without Cursor's Agent mode.

However, the k1 hallucination incident (Section 2) revealed a critical failure mode: AI can fabricate specific numerical values and weave them into plausible causal narratives. Because `k1 = -0.189` fit the story being constructed, I might have accepted it without verification.

My key takeaway: **AI excels at "how" but struggles with "why."** It efficiently implemented the Virtual IMU code (how to transform IMU data), but could not reason about why VIO failed from first principles. It could generate calibration configs, but fabricated causal explanations for results. Next time, I would enforce a strict rule: any numerical value cited by AI must be traced to a source file before acceptance, and I would be more skeptical of AI's causal reasoning about experimental outcomes.

---

## Section 7: Evidence

### 7.1 Code Snippet: AI Hallucination Correction

```yaml
# AI's fabricated claim (WRONG — this value does not exist in any file):
# "The HKisland config uses k1 = -0.189, causing ATE = 215m"

# Actual values from calibration files:
# calib_yaml/HK_GNSS(airport & island).yaml → k1 = -0.0560
# calib_yaml/HKisland.yaml                  → k1 = -0.0530
# calib_yaml/AMtown.yaml                    → k1 = -0.1210

# Root cause of 215m ATE: wrong fx/fy/cx/cy (HKisland intrinsics on AMtown data),
# NOT k1 difference.
```

### 7.2 k1 Ablation Results (Terminal Output)

```
k1=-0.121:  ATE RMSE = 110.7 m   (46× worse)
k1=-0.070:  TRACKING FAILURE      (crash)
k1=-0.056:  ATE RMSE = 5.25 m    (2.2× worse)
k1=-0.053:  ATE RMSE = 2.39 m    (optimal)
k1=-0.045:  ATE RMSE = 6.45 m    (2.7× worse)
k1=-0.035:  ATE RMSE = 22.1 m    (9.3× worse)
```

### 7.3 Prompt Example: Successful AI Use

```
Me: "Run Config D 5 times to pick the best result for AMtown02."
Cursor: [Autonomously created 5 sequential runs of mono_euroc, collected all
CameraTrajectory.txt outputs, ran evo_ape on each, compared ATE RMSE,
identified Run 2 (6.114m) as best, and saved the trajectory file.]
```

This was an ideal AI use case — repetitive, well-defined execution requiring no creative judgment.

### 7.4 Assignment 2: Source Code Modification

```cpp
// Tracking.cc — Lowered inlier threshold for sparse aerial scenes
// Before (default): if(mnMatchesInliers < 30)
if(mnMatchesInliers < 20)  // Tolerate fewer matches without triggering map reset
```
