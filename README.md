# COMPUTATIONAL INTELLIGENCE  

### Mini Project Repository

---

## 👨‍🎓 Student Information
- MUHAMMAD HAZIMUDDIN BIN FAIRUZ 
- MUHAMMAD ZAMARUL AZIM BIN ZULKHIBRI 
- MUHAMMAD AIMAN ALFARUQ BIN NOREDAN 

---

## 🧠 Project Title
**Mobile Robot Navigation Using Hybrid Fuzzy Logic and Genetic Algorithm**

---

## 📌 Introduction
Mobile robot navigation refers to the ability of a robot to move autonomously from a starting location to a target destination while safely avoiding obstacles in its environment. This capability is a fundamental requirement in many modern robotic applications, particularly in dynamic, uncertain, or cluttered environments.

---

## 🌍 Real-World Applications
Mobile robot navigation is widely applied in real-life systems such as:
- Indoor service robots (cleaning robots, delivery robots, hospital assistant robots)
- Automated Guided Vehicles (AGVs) in warehouses and factories
- Search and rescue robots in hazardous or inaccessible environments
- Autonomous vehicles operating in structured and semi-structured environments

These applications often involve uncertainty, incomplete sensor information, and changing surroundings, making traditional model-based control approaches difficult to implement.

---

## 🧠 Suitability of Fuzzy Logic
Fuzzy Logic is well-suited for mobile robot navigation because it mimics human-like reasoning using linguistic rules rather than precise mathematical models. It effectively handles uncertainty, imprecision, and noisy sensor data. In addition, fuzzy controllers are relatively simple to design and do not require an accurate kinematic or dynamic model of the robot.

---

## 🔗 Motivation for Hybrid Fuzzy–Genetic Algorithm
While fuzzy logic provides flexibility, its performance strongly depends on the design of membership functions and rule parameters, which are typically tuned manually. Genetic Algorithms (GA), inspired by natural evolution, are powerful optimization techniques that can automatically optimize these parameters.

By combining fuzzy logic with GA, the hybrid controller improves navigation performance, enhances adaptability, and reduces reliance on manual tuning.

---

## 📌 Project Overview
This project focuses on designing and simulating an intelligent mobile robot navigation system using a hybrid Fuzzy Logic and Genetic Algorithm approach. A point robot navigates within two-dimensional environments containing obstacles, with the objective of reaching a target location efficiently while avoiding collisions.

The performance of a basic fuzzy controller is compared with a hybrid fuzzy–GA controller across different map configurations.

---

## 🏗️ System Architecture and Environment Setup
The simulation environment and navigation controllers were developed using **Python** within the **PyCharm Integrated Development Environment (IDE)**. Python was selected due to its flexibility, extensive library support, and suitability for rapid prototyping of computational intelligence algorithms. PyCharm facilitated efficient code development, debugging, and visualization throughout the simulation process.

---

## 🛠️ Software Packages and Requirements
The following Python libraries were used:
- **NumPy** – numerical computation and vector operations  
- **Matplotlib** – visualization of robot paths and results  
- **Pygame** – real-time 2D simulation and animation  
- **scikit-fuzzy** – implementation of fuzzy logic systems  

---

## 📂 Repository Structure
