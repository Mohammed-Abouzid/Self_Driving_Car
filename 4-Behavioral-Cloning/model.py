#import csv
import numpy as np
import cv2
import tensorflow as tf
from keras.models import Sequential
from keras.layers import Conv2D, Activation, Flatten, Dense, Lambda, Cropping2D, Dropout
from keras.layers.pooling import MaxPooling2D
from keras.layers.advanced_activations import ELU
from keras.optimizers import Adam

#from sklearn.utils import shuffle
#from sklearn.model_selection import train_test_split
from utils import load_data

# preprocessing the data
images = []
measurements = []

load_data(images, measurements, 'driving_log', 'IMG')


##augment the dataset
augmented_imgs, augmented_measurs= [], []
for img, measur in zip(images, measurements):
    augmented_imgs.append(img)
    augmented_measurs.append(measur)

    augmented_imgs.append(cv2.flip(img, 1))    # or use image_flipped = np.fliplr(image)
    augmented_measurs.append(measur*-1.0)
x_train = np.array(augmented_imgs)
y_train = np.array(augmented_measurs)



# building the NN
model = Sequential()
model.add(Lambda(lambda x: x/255.0-0.5, input_shape=(160, 320, 3)))
model.add(Cropping2D(cropping=((70, 25), (0, 0))))

model.add(Conv2D(16, (5, 5), padding="valid"))
model.add(ELU())
model.add(MaxPooling2D((2, 2)))  # max pooling with filter 2x2

model.add(Conv2D(32, (3, 3), padding="valid"))
model.add(ELU())
model.add(MaxPooling2D((2, 2)))  # max pooling with filter 2x2

model.add(Conv2D(64, (3, 3), padding="valid"))
model.add(ELU())
model.add(MaxPooling2D((2, 2)))  # max pooling with filter 2x2


model.add(Flatten())
model.add(Dense(128))
model.add(ELU())
model.add(Dropout(0.4))

model.add(Dense(64))
model.add(ELU())
model.add(Dropout(0.5))

model.add(Dense(10))
model.add(ELU())
model.add(Dropout(0.5))

model.add(Dense(1))
model.add(ELU())

# train the model
adam = Adam(lr=0.0001)
model.compile(loss='mse', optimizer=adam, metrics=['accuracy'])
model.fit(x_train, y_train,  validation_split=0.25, shuffle=True, epochs=4)

model.save('model_last.h5')
