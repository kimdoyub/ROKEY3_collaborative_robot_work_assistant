
<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" />
  <img src="https://img.shields.io/badge/Python-3.10-yellow?logo=python" />
  <img src="https://img.shields.io/badge/License-Apache%202.0-blue.svg?logo=apache" />
</p>

# MyJarvice (Implementation of a Collaborative Robot Work Assistant Based on AI (Computer Vision))  

**Jarvis, the smart AI robot that handles the busy modern person's morning routine** **from Doosan Robotics Rokey Bootcamp3 in 2025**

---

## 🗂️ Overview
This project is designed to assist busy and tired modern individuals with their morning routines by providing breakfast, bed arrangement, and LLM-based services. For breakfast, it prepares coffee and cereal. For bed arrangement, it returns the blanket and pillow to their original positions. The LLM service provides weather updates and music recommendations based on the weather. Additionally, an exception-handling function was implemented to respond to unexpected situations.

---

## 🛠️ Equipment and Materials Used
![Equipment and Materials](images/materials.png)
- Doosan Collaborative Robot M0609  
- ROS2 Humble + Ubuntu 22.04

---
## 🧠 Assumptions for Scenario
1. Coffee beans for brewing drip coffee are assumed to be stored in a coffee can.
2. Due to constraints, the opening and closing of the cereal and milk containers are assumed to be done automatically.
3. The M0609 workbench is assumed to serve as both a cooking area and a bedroom.
---

## 📖 Scenario
![Assumptions](images/assumptions.png)
1. The LLM model is activated and voice recognition is performed using the wake-up phrase "Hey Jarvis."
2. At this stage, the user can choose the flavor of the coffee (bitter, nutty, or caramel) and the type of cereal (Chex Choco or Corn Frost).
3. Coffee and cereal are prepared according to the given commands.
4. After that, bed arrangement is carried out.
---

## 🎥 Demo Video
<p align="center">
  <a href="https://youtu.be/UUnwcfdMLlQ">
    <img src="https://img.youtube.com/vi/UUnwcfdMLlQ/0.jpg" alt="Watch the video" width="600"/>
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

