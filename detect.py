import time
from pyfirmata2 import Arduino
import cv2
import math
from ultralytics import YOLO

# Initialize the Arduino board and pins
board = Arduino('/dev/xyz')  # Replace with your port

middle = board.get_pin('d:9:s')  # Servo on pin 9
thumb = board.get_pin('d:13:s')
index = board.get_pin('d:11:s')

# Track the last grip time to ensure it lasts at least 3 seconds
last_grip_time = time.time()

def grip(strength) -> None:
    """
    Move all fingers to the specified strength.
    Args:
        strength (int): The strength to set for each finger (scaled 18-180). (10 strength = 180 degrees on servo motor)
    """
    thumb.write(strength)
    index.write(strength)    # Should be adjusted to 5 fingers in the future
    middle.write(strength)

def predict(chosen_model, img, classes=[], conf=0.5): # Bias should be higher the more precise the model becomes

    results = chosen_model.predict(img, classes=classes, conf=conf) if classes else chosen_model.predict(img, conf=conf)
    return results

def calculate_distance(box1, box2):
    # Distance between bounding boxes
    center1 = ((box1[0] + box1[2]) / 2, (box1[1] + box1[3]) / 2)
    center2 = ((box2[0] + box2[2]) / 2, (box2[1] + box2[3]) / 2)
    return math.sqrt((center1[0] - center2[0])**2 + (center1[1] - center2[1])**2)

# Map objects to grip strength
# IMPORTANT: limited dataset used for this project (limited time)
# Use more general dataset to include variety of objects, but might need to hard-code grip strength values anyway
grip_strength_map = {
    "hand": 8,
    "phone": 10,
    "can": 10
}

def find_closest_object_and_grip(results, reference_class_name="prosthetic", range_threshold=400, last_ref_box=None):
    """
    Find the closest object to the prosthetic and determine grip strength,
    including cases where both the prosthetic and object are detected.
    """
    grip_strength = 0
    reference_box = None

    # Locate the prosthetic's bounding box
    for box in results[0].boxes:
        class_id = int(box.cls[0])
        class_name = results[0].names[class_id]
        if class_name == reference_class_name:
            reference_box = box.xyxy[0].tolist()
            break

    if reference_box is None:
        if last_ref_box is not None:
            reference_box = last_ref_box
        else:
            return grip_strength, last_ref_box

    last_ref_box = reference_box
    closest_distance = float("inf")
    closest_object = None

    # Find the closest object to the prosthetic
    for box in results[0].boxes:
        class_id = int(box.cls[0])
        class_name = results[0].names[class_id]

        # Skip the prosthetic itself (in case of double detection)
        if class_name == reference_class_name:
            continue

        # Calculate distance to the prosthetic
        object_box = box.xyxy[0].tolist()
        distance = calculate_distance(reference_box, object_box)

        # Check if object is within range and closer than the previous closest object
        if distance <= range_threshold and distance < closest_distance:
            closest_distance = distance
            closest_object = class_name

    # If the prosthetic and object overlap, use object's grip strength
    if closest_object and closest_object in grip_strength_map:
        grip_strength = grip_strength_map[closest_object]

    return grip_strength, last_ref_box

def predict_and_detect(chosen_model, img, classes=[], conf=0.4, rectangle_thickness=3, text_thickness=2, last_ref_box=None):
    """
    Perform prediction and handle grip strength calculations.
    """
    global last_grip_time
    results = predict(chosen_model, img, classes, conf=conf)
    grip_strength, last_ref_box = find_closest_object_and_grip(results, reference_class_name="prosthetic", range_threshold=400, last_ref_box=last_ref_box)

    arduino_strength = grip_strength * 18
    current_time = time.time()
    if grip_strength > 0:  # Grip when strength is non-zero
        if current_time - last_grip_time >= 3:  # Grip lasts at least 3 seconds
            grip(arduino_strength)
            last_grip_time = current_time
    else:  # Release grip when no object is detected
        grip(0)  # Reset grip strength to zero

    # Box design
    for result in results:
        for box in result.boxes:
            cv2.rectangle(img, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                          (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (0, 0, 255), rectangle_thickness)
            class_id = int(box.cls[0])
            class_name = result.names[class_id]
            confidence = box.conf[0]
            cv2.putText(img, f"{class_name}: {confidence:.2f}",
                        (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                        cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), text_thickness)

    return img, grip_strength, last_ref_box


# Main loop
last_ref_box = None
model = YOLO("/path/to/your/model.pt")
cap = cv2.VideoCapture(0)

fps = 30 # Depends on camera
cap.set(cv2.CAP_PROP_FPS, fps)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    result_img, grip_strength, last_ref_box = predict_and_detect(model, frame, classes=[], conf=0.4, last_ref_box=last_ref_box)
    cv2.putText(result_img, f"Grip Strength: {grip_strength}",
                (10, 50), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Real-Time Object Detection", result_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
