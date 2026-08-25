# Soft-Gripper-Etextiles 

**Development of an underactuated soft robotic gripper integrated with e-textile tactile sensors.** 

This repository contains the hardware designs, source code, data, and documentation for an engineering internship project conducted at the **Tyndall National Institute** (MicroNano Systems / WSN Group) in Cork, Ireland.

## 📝 Project Overview

Handling fragile objects safely is a complex challenge in robotics. Traditional industrial grippers are often too rigid and consume continuous power to maintain their grip, which wastes energy. 

This project introduces a **3D-printed soft gripper** designed to solve these issues:
1. **Zero-Energy Holding:** Utilizes a custom self-locking worm screw mechanism that maintains a firm grip on objects without consuming electrical power (0 Watts).
2. **Compliant Design:** Underactuated (driven by a single servo motor) with flexible 3D-printed fingers that conform to the shape of heterogeneous and fragile objects.
3. **Smart Tactile Feedback:** Integration of embroidered e-textile sensors (Shieldex 117/17 conductive yarn) on the fingertips for contact detection.

The gripper was rigorously tested against a standard rigid rack-and-pinion mechanism using an OptiTrack motion capture system, load cells, and SimScale FEA for geometric optimization.

## ✨ Key Features

* **Self-Locking Drive:** Worm screw transmission prevents back-driving.
* **Low-Cost & Rapid Assembly:** Costs under 2€, composed of only 2 main 3D-printed parts, assembling in under 3 minutes.
* **Optimized Geometry:** Analyzed via Euler-Bernoulli beam theory and FEA to prevent mechanical buckling and maximize actuation efficacy (up to 14.68 N/W).
* **E-Textile Integration:** Smart fabric patches embroidered with a serpentine pattern for strain/resistance measurement and contact detection.
* **Robotic Arm Ready:** Includes a custom 3D-printed adapter for mounting on standard cobots.

## 🗂️ Repository Structure

* 📁 **`Arduino_Code/`** : Scripts for controlling the Parallax continuous servo motor and reading the e-textile sensor data via a Wheatstone bridge.
* 📁 **`CAD/`** : 3D models (STEP/STL/3MF) of the compliant gripper, the rigid reference gripper, testing setups, and the robotic arm adapter.
* 📁 **`Certificates/`** : Mandatory laboratory safety, cybersecurity, and equipment training certificates completed during the internship.
* 📁 **`E-textiles/`** : Files related to the e-textile sensors (embroidery patterns, characterization data, and images).
* 📁 **`Literature/`** : State-of-the-art research review on soft robotics, self-locking mechanisms, and conductive yarns.
* 📁 **`MatLab_Code/`** : Scripts used for real-time plotting, data logging, and generating the kinematic, hysteresis, and mechanical drift graphs.
* 📁 **`Results/`** : Raw data and experimental results from force, current, power, and OptiTrack kinematics testing.
* 📁 **`Weekly reports/`** : Presentation slides tracking the project's progress and weekly R&D updates.

## 🛠️ Hardware & Software Used

* **Fabrication:** Prusa MK4S 3D Printer (Prusament PLA), Brother Innov-is NV2700 Embroidery Machine.
* **Electronics:** Arduino UNO R4 WiFi, Parallax continuous rotation servo, HX711 amplifier, AD623 instrumentation amplifier, 0-50N Load Cell.
* **Software:** Onshape (CAD), PrusaSlicer, Arduino IDE, MATLAB, SimScale (FEA), Motive (OptiTrack), Ink/Stitch (Embroidery).

## 👨‍💻 Author

**Julien Dubuc**  
Engineering Student at SeaTech (Université de Toulon) - SYSMER Track.

## 🙏 Acknowledgements

Special thanks to my supervisor, **Matteo Menolotto**, for his guidance, and to the entire WSN group at the Tyndall National Institute and University College Cork (UCC) for their support and access to the laboratory facilities.
