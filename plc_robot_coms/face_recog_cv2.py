# standard face recognition using cv2 python3
# pip install --upgrade pip; pip install opencv-python
import cv2

# set this to the path you have the xml file 
# this was my linux you windows might be like C:/Users/user/AppData/Local/Programs/Python/Python37/Lib/site-packages/cv2/data
face_cascade = cv2.CascadeClassifier(r'/home/mark/.local/lib/python3.6/site-packages/cv2/data/haarcascade_frontalface_default.xml')
cap = cv2.VideoCapture(0)

while True:
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
    for x, y, w, h in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        face = img[y: y + h, x: x + w]
        face_gray = gray[y: y + h, x: x + w]
        cv2.putText(img, 'Face Detection', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,200), 2, cv2.LINE_AA)
        cv2.imshow('video image', img)
        key = cv2.waitKey(10)
        if key == 27:                # To stop the program, press the “Esc” key.
            break
    cap.release()
    cv2.destroyAllWindows()