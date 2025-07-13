# Vision-Based Adaptive Soft Gripper

## Project Overview

This project presents an adaptive soft gripper system capable of detecting object shapes using computer vision and estimating object dimensions through 3D point cloud analysis. Based on the detected shape and size, the system automatically adjusts the gripper’s orientation.

The setup uses a RGBD camera to fuse color and depth information, allowing real-time classification of common shapes (e.g., circle, square, rectangle) and dimension estimation from aligned depth data.

### Features

- Vision-based shape detection using OpenCV
- Dimension estimation from aligned RGB-D data using point cloud processing via the RealSense SDK
- Real-time 3D visualization with Open3D
- Real-time feedback from a bending sensor integrated into the soft gripper for monitoring deformation

-----

## 🎥 Full System Demonstration

**Watch the complete system in action:**

[![Vision-Based Adaptive Soft Gripper Demo](https://img.shields.io/badge/▶️_Watch_Full_Demo-Google_Drive-4285F4?style=for-the-badge&logo=googledrive&logoColor=white)](https://drive.google.com/file/d/13DtPGQm4SXABxC2uEJvvoZYxTlOsWo28/view?usp=sharing)

*Click the button above to view the comprehensive demonstration video showcasing the gripper’s adaptive capabilities across different object shapes and sizes.*

-----

## Gripper Actuation Mechanism

<p align="center">
  <img src="images\Soft_Gripper_Prototype.PNG" width="300"/>
</p>

This rotary-linear mechanism controls the gripper’s opening based on servo angles. Three configurations are used:

- **Spherical**: All servos at 90° for round objects.
- **Cylindrical**: Rear servos at 60° and 120°, front at 90°.
- **Parallel**: Rear servos at 145° and 30°, front at 90° for flat surfaces.

These configurations allow adaptive grasping based on object geometry.

-----

## Vision-Based Control Setup

<p align="center">
  <img src="images/Project_Setup.jpg" width="300"/>
</p>

The Intel RealSense D435i camera is positioned in front of the gripper, facing the object. It captures synchronized RGB and depth data used for shape detection and size estimation. The camera is fixed in place to maintain a consistent field of view during operation.

-----

## Object Shape Detection and Size Estimation Demonstration

<table>
  <tr>
    <td align="center">
      <img src="images/Circle.gif" width="400"/><br/>
      <strong>Circle-Shaped Object</strong>
    </td>
    <td align="center">
      <img src="images/Rectangle.gif" width="400"/><br/>
      <strong>Rectangle-Shaped Object</strong>
    </td>
  </tr>
  <tr>
    <td colspan="2" align="center" style="padding-top: 20px;">
      <img src="images/Square.gif" width="400"/><br/>
      <strong>Square-Shaped Object</strong>
    </td>
  </tr>
</table>

### Key Capabilities Demonstrated

- **Real-time Shape Recognition**: Automatic detection of circular, rectangular, and square objects
- **Adaptive Gripper Configuration**: Dynamic adjustment of servo positions based on detected geometry
- **3D Dimension Analysis**: Accurate size estimation using depth data for optimal grasping strategy
- **Integrated Feedback**: Continuous monitoring through embedded bending sensors

-----

## Technical Implementation

The system integrates multiple technologies to achieve autonomous adaptive grasping:

1. **Computer Vision Pipeline**: RGB image processing for shape classification
1. **Depth Analysis**: Point cloud processing for dimensional measurements
1. **Servo Control**: Precise actuation based on object characteristics
1. **Sensor Feedback**: Real-time deformation monitoring during grasping operations
