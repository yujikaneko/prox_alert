import cv2
import os

save_directory = 'calib'
if not os.path.exists(save_directory):
    os.makedirs(save_directory)

cap = cv2.VideoCapture(0)
width = 1920
height = 1080
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    img = cv2.resize(frame, (width//2, height//2))
    cv2.imshow("image", img)
    
    key = cv2.waitKey(1)    
    if key == ord('q'):
        break
    elif key == ord('s'):
        save_path = os.path.join(save_directory, f"chessboard_{count}.jpeg")
        cv2.imwrite(save_path, frame)
        print(f"Saved image to {save_path}")
        count += 1

cap.release()
cv2.destroyAllWindows()
