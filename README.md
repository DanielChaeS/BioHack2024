# BioHack2024
Cost-Efficient AI Prosthetic - Team 18

Biohack Prosthetic Code

Overview:
This project is designed to enhance prosthetic hand functionality by detecting objects using a camera and adjusting grip strength accordingly. The system uses YOLOv11 for object detection, processes the input through Python scripts, and communicates with a microcontroller via Bluetooth to control grip strength.

Features:
1. Object Detection: Identifies objects in real time using YOLOv11.
2. Grip Strength Adjustment: Determines the necessary grip strength based on the detected object.
3. Arduino Compatibility for 3rd Party Cameras: Sends grip strength data to the prosthetic hand controller.

Requirements:

pip install -r requirements.txt

YOLO CNN is trained (datasets and training notebook to be added)

Arduino IDE with relevant code (to be added)

3rd Party Camera connected to Arduino



Usage:

python detect.py

This will open the camera feed and begin real-time object detection.

The detected object near the prosthetic will be classified, and an appropriate grip strength value will be calculated.

Value goes through Arduino to adjust Servo motors on prosthetic


The program outputs logs for detected objects, grip strength values, and transmission status.




Future Improvements

Implement more object categories for enhanced accuracy.

Optimize grip strength calculations based on real-world testing.

Improve latency for faster real-time processing.



License

This project is licensed under the MIT License - see the LICENSE file for details.
