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
Install dependencies:

pip install -r requirements.txt

Ensure YOLOv11 is properly set up and trained if needed.

Connect the camera and ensure it is accessible by OpenCV.

Pair the microcontroller via Bluetooth and verify the connection.

Usage

Start Object Detection

python detect.py

This will open the camera feed and begin real-time object detection.

Grip Strength Calculation & Transmission

The detected object will be classified, and an appropriate grip strength value will be calculated.

This value is transmitted via Bluetooth to the prosthetic hand controller.

Monitor Logs

The program outputs logs for detected objects, grip strength values, and transmission status.

Troubleshooting

YOLO Model Not Detecting Objects?

Ensure the model is correctly loaded and trained.

Verify OpenCV is accessing the camera properly.

Bluetooth Connection Issues?

Check if the microcontroller is paired with the computer.

Ensure the correct COM port or device ID is specified.

Performance Lag?

Optimize YOLO model settings (lower resolution, batch size, etc.).

Use an NVIDIA GPU for CUDA acceleration.

Future Improvements

Implement more object categories for enhanced accuracy.

Optimize grip strength calculations based on real-world testing.

Improve latency for faster real-time processing.

License

This project is licensed under the MIT License - see the LICENSE file for details.

Contributors

Daniel Chae (Project Lead & Developer)

Team Members (if applicable)

For questions or suggestions, contact your.email@example.com or open an issue on the repository.

