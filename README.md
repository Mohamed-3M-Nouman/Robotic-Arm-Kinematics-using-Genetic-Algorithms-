# Robotic Arm Kinematics using Genetic Algorithms 🤖🧬

An interactive Python simulation that leverages a **Genetic Algorithm (GA)** to solve the **Inverse Kinematics (IK)** problem for a multi-link robotic arm. The project demonstrates how evolutionary computation can be used to find optimal joint angles for a robotic arm to reach a specified 2D target point smoothly and accurately, without relying on complex analytical matrix inversions.

## 💡 The Core Idea: Why Genetic Algorithms?

### The Problem: Inverse Kinematics

In robotics, **Forward Kinematics** is straightforward: given the angles of the joints, calculate where the tip of the arm (end-effector) will end up. However, **Inverse Kinematics (IK)** is much harder: given a desired target coordinate (x, y), calculate the exact angles each joint must take to reach it.Traditional mathematical solutions suffer from singularity issues, require heavy computational matrix operations, and become incredibly complex as the number of joints increases.

### The Solution: Evolutionary Computation

This project bypasses the analytical math by using a **Genetic Algorithm**, treating the IK problem as an optimization task. The algorithm mimics biological evolution to "evolve" the perfect set of angles:

1.  **Chromosome Representation:** Each "individual" is an array of floating-point numbers representing the angles of the robotic joints.
2.  **Fitness Function:** Evaluated based on the Euclidean distance between the arm's end-effector and the target point.
3.  **Selection & Crossover:** Selects the best solutions and mixes their angles to produce a new, better "child" solution.
4.  **Mutation:** Adds slight random variations to prevent the algorithm from getting stuck in local optima.

## ✨ Key Features

- **Two Execution Modes:** Interactive Web Dashboard (Streamlit) and Local CLI Mode (Matplotlib).
- **Dynamic Arm Configuration:** Define an arm with any number of joints of varying lengths.
- **Customizable GA Parameters:** Control Population Size, Mutation Rate, and Max Generations on the fly.
- **Real-time Animation:** Visualizes the evolutionary process generation by generation.

## 🛠️ Tech Stack

- **Python 3.x**
- **NumPy:** For vectorized mathematical operations and logic.
- **Matplotlib:** For rendering the 2D kinematics and animations.
- **Streamlit:** For building the interactive web frontend.

## 🚀 How to Run (Installation & Usage)

### 1\. Install Dependencies

Clone this repository and install the required libraries by running:

`   pip install numpy matplotlib streamlit   `

### 2\. Method A: Run the Web Dashboard (Recommended)

You can run the application as a modern web app using Streamlit.

- Double-click the Run_Web.bat file.
- **OR** execute this command in your terminal:

`   streamlit run "Robotic Arm Kinematics GA.py"   `

### 3\. Method B: Run the Local CLI Simulation

If you prefer the command-line interface:

- Double-click the Run_Local.bat file.
- **OR** execute this command in your terminal:

`   python "Robotic Arm Kinematics GA.py"   `

_(Note: In CLI mode, you must close the matplotlib animation window to proceed to the next prompt or to exit)._

## 📊 Presentation

For a deeper dive into the project's logic, architecture, and results, check out the full presentation:👉 [**Click here to view the Presentation**](https://docs.google.com/presentation/d/1diYG_Ce8qukeutnBhkHc77YPHJd4hff7/edit?usp=sharing&ouid=101866060512381107780&rtpof=true&sd=true)

## 👨‍💻 Author

**Mohammed Nouman ELghareb** | Artificial Intelligence Engineer & Data Analyst
"# Robotic-Arm-Kinematics-using-Genetic-Algorithms-" 
