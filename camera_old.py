import cv2
import numpy as np
 
def main():
    lower_bound = np.array([0,132,61])
    upper_bound = np.array([14,255,255])
 
    cap = cv2.VideoCapture(0)
 
    if not cap.isOpened():
        print("Error: Camera not accessible")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            print(f"Contour Area: {cv2.contourArea(contour)}")

        filtered_contours = [x for x in contours if cv2.contourArea(x) > 100 and cv2.contourArea(x) < 30000]

        contours = filtered_contours

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            # x, y, w, h = cv2.boundingRect(largest_contour)
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(frame, center, radius, (0, 255, 0), 2)
 
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
 
    cap.release()
    cv2.destroyAllWindows()
 
if __name__ == "__main__":
    main()