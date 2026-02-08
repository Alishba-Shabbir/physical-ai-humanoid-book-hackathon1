---
sidebar_position: 5
---

# Capstone Project: Autonomous Humanoid

## Overview

The final project tasks you with building a simulated autonomous humanoid that accepts spoken instructions, plans and navigates a path through a cluttered environment, identifies objects using computer vision, and manipulates them safely. The project integrates the course's major themes: Physical AI, ROS 2 middleware, simulation (Isaac/Gazebo/Unity), perception, planning, and embodied action.

Key deliverables:

- A simulated humanoid platform running in Isaac Sim, Gazebo, or Unity.
- A voice-to-action pipeline that converts spoken commands to executable tasks.
- A navigation and footstep planning solution (Nav2 + footstep planner) that handles dynamic obstacles.
- An object detection and pose-estimation module that enables pick-and-place manipulation.
- A documented sim-to-real validation plan and set of test logs.

## Learning Objectives

- Apply Physical AI concepts in a complete, end-to-end simulation.
- Integrate ROS 2 nodes and topics with NVIDIA Isaac or another simulator.
- Implement a Vision‑Language‑Action (VLA) pipeline: speech → intent → plan → action.
- Design and evaluate autonomous planning, perception, and manipulation pipelines for a humanoid.

## Project Steps

1. Voice‑to‑Action

	- Capture audio from the robot's microphone and transcribe it with OpenAI Whisper or an equivalent ASR system.
	- Parse the transcription to extract an intent and parameters (e.g., object, location, constraints).

2. Path Planning

	- Use Nav2 for high‑level path planning (global plan) over an occupancy map.
	- Implement a footstep planner that converts Nav2 waypoints into bipedal foot placements and timings.

3. Obstacle Navigation

	- Simulate dynamic obstacles and implement collision avoidance and local replanning.
	- Integrate sensor feedback (LiDAR, depth camera) to update costmaps and trigger replanning.

4. Object Detection

	- Use RGB-D cameras and a pretrained detector (YOLO, Detectron2, or TensorRT-optimized model) to find candidates.
	- Estimate object pose (PCL, ICP, or learning-based pose estimators) for grasp planning.

5. Manipulation

	- Implement grasp planning and whole‑body motion planning to reach, grasp, and place objects while preserving balance.
	- Provide safe fallback behaviors (stow, abort, replan) on perception or actuator failures.

6. Testing & Sim‑to‑Real Transfer

	- Validate pipelines in simulation, collect logs, and identify mismatch with real-world traces.
	- Apply sim-to-real strategies: domain randomization, system identification, and incremental hardware testing with safety interlocks.

## Example Code Snippets

Voice capture and Whisper transcription (Python, simplified):

```python
import whisper
import soundfile as sf

model = whisper.load_model('small')
audio, sr = sf.read('recording.wav')
result = model.transcribe('recording.wav')
text = result['text']
print('Transcribed:', text)
```

Mapping transcription to ROS 2 actions (pseudo-code):

```python
# subscribe to /vla/transcription
def on_transcription(text):
	 intent = parse_intent(text)
	 plan = planner.expand(intent)
	 for step in plan:
		  call_ros2_action(step)

def call_ros2_action(step):
	 # create action client, send goal, wait for result
	 pass
```

Publishing camera frames and receiving detections (ROS 2 topics example):

```bash
# run simulator that publishes /camera/rgb/image_raw and /camera/depth/image_raw
# run detector node that subscribes and publishes /perception/detections
# visualize in RViz2
ros2 topic list
ros2 topic echo /perception/detections
```

## Tips and Example Diagrams (describe in text)

- Path planning diagram: show Nav2 producing a global 2D plan overlaid on an occupancy grid; arrows from Nav2 feed into a footstep planner that outputs discrete footstep positions; the whole-body controller converts footsteps into joint trajectories while monitoring CoM and contact forces. Annotate feedback loops for replanning and slip detection.

- Sensor setup diagram: illustrate camera, LiDAR, IMU placements on the humanoid (head-mounted RGB-D camera, torso IMU, LiDAR at chest height). Annotate topics for each sensor and expected message rates.

- VLA pipeline diagram: chain showing audio input → ASR (Whisper) → NLU/intent parser → Task planner → ROS 2 actions (navigation, perception, manipulation) with feedback arrows from perception and controllers.

Practical tips:

- Run Whisper or heavy models on a GPU node; avoid blocking control loops with synchronous inference.
- Use separate processes or composable nodes for perception and control to isolate failures.
- When training detectors on synthetic data, include domain randomization (lighting, textures, object scale) and fine-tune with a small real dataset.
- Log timestamps for audio, sensors, and control commands to diagnose latency and timing mismatches.

## Troubleshooting Common Simulation Errors

- Robot falls immediately on spawn:
  - Check collisions: overlapping collision meshes or interpenetrating links often cause instabilities. Simplify collision geometry and re-run.
  - Verify initial pose and center-of-mass: correct URDF mass/inertia values and initial joint positions.

- Sensors publish empty or noisy data:
  - Inspect sensor topic QoS and ensure the simulator bridge exposes messages with compatible QoS settings.
  - Reduce resolution or sampling rate to identify performance bottlenecks.

- High latency between perception and action:
  - Profile end-to-end pipeline (capture → inference → publish → action). Consider batching, TensorRT, or moving inference to a dedicated GPU.

- Mismatch between sim and real robot behavior:
  - Run system identification: tune friction coefficients, actuator delays, and mass values.
  - Use incremental testing on hardware with safety limits (reduced gains, speed limits).

## Evaluation & Grading Suggestions

- Functional tests: a set of scripted tasks (pick-and-place, obstacle course) the robot must complete in simulation.
- Robustness metrics: success rate under domain randomization, recovery from perception dropouts, and time-to-complete.
- Performance metrics: end-to-end latency, perception precision/recall, planning time, and energy (simulated) consumed.

## Submission Checklist

- Source code for ROS 2 nodes and launch files (organized in a package).
- Simulator scenes and URDF/xacro files used in experiments.
- Recorded logs: bag files (or equivalent), sample sensor data, and video captures of runs.
- A short report describing architecture, parameter choices, sim-to-real steps, and failure analysis.

---

Proceed to the appendix for tooling, installation instructions, and extra resources.
