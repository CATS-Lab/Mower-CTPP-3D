# Coverage trajectory planning problem on 3D terrains with safety constraint for automated lawn mower: Exact and heuristic approaches

## Introduction

This repository contains the source code and data for the following paper:

**Zhou, H., Zhang, P., Liang, Z., Li, H., \& Li, X.** (2025). *Coverage trajectory planning problem on 3D terrains with safety constraint for automated lawn mower: Exact and heuristic approaches.*  Robotics and Autonomous Systems..

Autonomous mowing systems offer the potential to enhance efficiency and reduce labor in lawn maintenance. However, the irregularities of real-world terrains such as slopes and obstacles introduce significant safety challenges that must be addressed in trajectory planning. This repository provides the implementation of a trajectory planning framework for automated lawn mowers operating on 3D terrains. It introduces the Coverage Trajectory Planning Problem on 3D Terrains (CTPP-3DT), which jointly optimizes the mower's path and speed profile to minimize completion time while satisfying safety constraints on sloped surfaces. The framework includes a Mixed-Integer Linear Programming model for small-scale instances and a decomposition-based heuristic algorithm using Simulated Annealing for large-scale problems. Benchmark experiments and sensitivity analyses are provided to evaluate the effectiveness and generalizability of the proposed approach.

## Usage

### Code

The project is developed using Java 1.8. Please ensure you have the corresponding environment set up.

The code related to our data processing and algorithm are included in folder `src`. As you proceed through all code, always verify the paths for both the input and output files. This ensures that everything runs smoothly.

### Data

The `data` folder contains the benchmark reference trajectories used in our paper. You can generate additional instances using `src/Main/InstanceGenerator.java`.

Each instance is stored as a `.txt` file with the following format:

```
map length
map width
(blank line)
obstacle map (0: grass, 1: obstacle, 2: mower start)
(blank line)
height map
```

## Developers

Developers - Hang Zhou (hzhou364@wisc.edu).

If you have any questions, please feel free to contact the CATS Lab at UW-Madison. We're here to help!
