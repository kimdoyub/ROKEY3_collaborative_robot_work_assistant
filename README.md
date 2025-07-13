
<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" />
  <img src="https://img.shields.io/badge/Python-3.10-yellow?logo=python" />
  <img src="https://img.shields.io/badge/License-Apache%202.0-blue.svg?logo=apache" />
</p>

# MyJarvice (Implementation of a Collaborative Robot Work Assistant Based on AI (Computer Vision))  

**Jarvis, the smart AI robot that handles the busy modern person's morning routine** **from Doosan Robotics Rokey Bootcamp3 in 2025**

---

## 🗂️ Overview
This project utilizes the Doosan collaborative robot M0609 to symbolically implement and prototype key construction functions commissioned by government or enterprise clients. The robot interprets architectural blueprints, performs construction tasks accordingly, and detects defective materials, offering a proof-of-concept system for automated building assistance.

---

## 🛠️ Equipment and Materials Used
![Equipment and Materials](images/materials.png)
- Doosan Collaborative Robot M0609  
- ROS2 Humble + Ubuntu 22.04

---
## 🧠 Assumptions for Scenario
![Assumptions for Scenario](images/assumptions.png)
1. There are multiple robots in real-world scenarios; however, only the M0609 is considered in this project.
2. Due to constraints, the steel bar plate is used to represent the blueprint.
3. Warehouses are built in sourcing areas, and all materials are classified by workers; therefore, misclassifications may occur.
4. There are three types of buildings.
5. The disposal area and truck mixer are represented by cups.
6. The building site is assumed to be a LEGO base.
---

## 📖 Scenario
1. The M0609 transports construction materials for foundation work and carries out concrete pouring.
2. The robot interprets the blueprint by measuring the height of steel bars embedded in the plate, saving information about the building type and its position.
3. Based on the interpreted data, the robot stacks LEGO blocks to build the structure.
- If the gripper detects an abnormal object, the robot moves it to the disposal area.
- Even if the robot stops due to an error or emergency, it can resume operation using system variables.
---
## 🎥 Demo Video

<p align="center">
  <a href="https://youtu.be/LTs1zqtvSvc">
    <img src="https://img.youtube.com/vi/LTs1zqtvSvc/0.jpg" alt="Watch the video" width="600"/>
  </a>
</p>

---

## 📄 Documentation

For a detailed explanation of this project, please refer to the following document:

👉 [doc/F-2_협동2_김도엽_이재호_이한용_손지훈.pdf](docs/F-2_협동2_김도엽_이재호_이한용_손지훈.pdf)

---

## 👥 Contributors

Thanks to these wonderful people who have contributed to this project:

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/kimdoyub">
        <img src="https://github.com/kimdoyub.png" width="100px;" alt="kimdoyub"/><br />
        <sub><b>kimdoyub</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/SmiteLims">
        <img src="https://github.com/SmiteLims.png" width="100px;" alt="SmiteLims"/><br />
        <sub><b>SmiteLims</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/machyong">
        <img src="https://github.com/machyong.png" width="100px;" alt="machyong"/><br />
        <sub><b>machyong</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/jihoonindices">
        <img src="https://github.com/jihoonindices" width="100px;" alt="jihoonindices"/><br />
        <sub><b>jihoonindices</b></sub>
      </a>
    </td>
  </tr>
</table>


---

