# Intelligent Robot Car with ESP32-CAM, Arduino UNO, and PC AprilTag Control

This project establishes a complete three-tier control system integrating an **ESP32-CAM** module, an **Arduino UNO** microcontroller, and a **Python-based PC control program**. Together, these components form a closed-loop intelligent vehicle platform capable of visual recognition, real-time wireless communication, and motion control. Each endpoint performs a distinct role while cooperating through structured data exchange in JSON format over both serial and HTTP protocols.

---

## System Overview

![System Architecture](https://github.com/StevenLiuMC/Auto-Tracking-Apriltag--Smart-Car/blob/main/System%20Architecture.png)

At the hardware level, the **ESP32-CAM**, developed under the **Visual Studio Code PlatformIO** environment, serves as the communication **pivot** and vision acquisition unit. It operates in **Access Point (AP)** mode, hosting an asynchronous web server that streams live video to the PC through an MJPEG stream endpoint (`/mjpeg`) and responds to control commands sent via HTTP POST requests to the `/cmd` endpoint. The camera module continuously captures frames using the `esp_camera` driver, compresses them in JPEG format, and broadcasts them to all connected clients in real time. The ESP32 also manages UART communication with the Arduino UNO, transmitting compact JSON commands that encapsulate motion instructions such as direction and speed. Each command is serialized through the `ArduinoJson` library, ensuring minimal overhead and structured data transfer.

The **Arduino UNO**, programmed through the **Arduino IDE**, receives these JSON packets over a serial link at 9600 bps. Upon receipt, it deserializes the data and interprets the motion directives to control the vehicle’s actuators via motor driver modules such as the **TB6612**. The UNO functions as the low-level executor of the system, responsible for precise hardware actuation and sensor integration. Its role is intentionally lightweight to maximize real-time responsiveness and maintain synchronization with the higher-level ESP32 pivot.

The **PC application**, implemented in **Python**, operates as the perception and decision-making unit. It utilizes the `pupil_apriltags` library to detect and localize AprilTags in the video stream provided by the ESP32. Through continuous frame processing, it computes the horizontal offset of the detected tag relative to the image center and determines whether the vehicle should move forward, turn left, or turn right. These motion decisions are transmitted back to the ESP32 using HTTP POST requests in JSON form, specifying the desired motion state and speed. This feedback forms the upper control loop of the system, closing the interaction chain between visual perception and actuation. The program also provides optional visualization through **OpenCV**, overlaying detection results and control status on the live video feed.

---

## System in Action

![Smart Car Picture](https://github.com/StevenLiuMC/Auto-Tracking-Apriltag--Smart-Car/blob/main/Whole%20Car.jpg)

The overall workflow unfolds as a seamless feedback loop: the **ESP32-CAM** captures visual data and broadcasts it wirelessly to the **PC**; the **Python controller** processes the frames to infer movement directives; these directives are relayed back to the **ESP32** via HTTP, serialized, and forwarded through **UART** to the **Arduino UNO**, which executes the physical motion. This architecture demonstrates a clean separation of concerns — the ESP32 handling network and camera operations, the PC performing vision-based reasoning, and the Arduino UNO conducting direct motor control — all connected by lightweight, human-readable JSON communication.

By uniting these components, the system achieves real-time visual servoing through a compact and modular design. It exemplifies an embedded–edge–host integration pipeline where perception, communication, and control cooperate efficiently to enable intelligent motion guided by visual cues.

---

## Kalman Filter Experiment

![Kalman Filter](https://github.com/StevenLiuMC/Auto-Tracking-Apriltag--Smart-Car/blob/main/Kalman%20Filter.png)

During development, a **Kalman Filter** was explored as part of the host computer’s vision pipeline to perform trajectory prediction for AprilTag motion. While the Kalman-based estimator demonstrated promising results in smoothing and predicting tag trajectories, it was determined that this additional filtering layer was not essential for achieving the project’s core functionality. Therefore, the Kalman filter experiment was retained as an **independent module** for future integration and testing, rather than being part of the main control loop.

---

## Summary

This project highlights how distributed computation across microcontrollers and a host computer can enable real-time perception-driven control.  
By combining **Python-based visual intelligence**, **ESP32 network bridging**, and **Arduino-level actuation**, the system demonstrates an effective and extensible architecture for autonomous behavior in embedded robotic applications.
