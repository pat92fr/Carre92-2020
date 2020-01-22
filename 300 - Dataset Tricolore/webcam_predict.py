from keras.models import load_model
import numpy as np

model = load_model('first_try.h5')
model.summary()
names = [ "Orange", "Rouge", "RougeOrange", "Vert" ] 
width = 1280//8
height = 720//8

import cv2

video = cv2.VideoCapture(1)

while True:
	_, frame = video.read()
	#print(frame.shape)
	frame_resized = cv2.resize(frame,(width,height))
	#print(frame_resized.shape)
	cv2.imshow("Capturing", frame_resized)
	prediction = model.predict(frame_resized.reshape(1,height,width,3))
	print(prediction)
	print( names[np.argmax(prediction, axis = None)] )
	key=cv2.waitKey(1)
	if key == ord('q'):
		break
	

video.release()
cv2.destroyAllWindows()