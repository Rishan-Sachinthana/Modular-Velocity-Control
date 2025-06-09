# my_package

This is a ROS 2 package for [brief description of what it does].
Modular Reconfigurable Robots (MRRs) can dock together in various morphological formations to meet task requirements. Once a formation is established, the MRR system should be capable of smooth locomotion. To achieve this, MRRs require a generalized kinematic model that supports locomotion in any given configuration. However, a ROS package that enables such functionality is currently unavailable. To address this gap, we have developed a new ROS package that provides locomotion kinematic modeling for MRRs, enabling movement in any morphological formation. The proposed model has been validated using the Smorphi MRR with various numbers of modules and shape formations.

## Requirements

- ROS 2 (e.g., Humble, Iron, etc.)
- colcon
- Python 3.x

## Build Instructions

```bash
cd ~/ros2_ws
colcon build --packages-select my_package
