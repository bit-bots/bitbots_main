#!/usr/bin/env python2.7
import os

import keras
import numpy as np
from keras.layers import Convolution2D, Flatten, Dense
from keras.layers import MaxPooling2D
from keras.models import Sequential
from keras.preprocessing.image import ImageDataGenerator, load_img, img_to_array
from matplotlib import  pyplot as plt


datagen = ImageDataGenerator(
        rotation_range=10,
        width_shift_range=0.1,
        height_shift_range=0.1,
        rescale=1./255,
        shear_range=0.1,
        zoom_range=0.1,
        horizontal_flip=False,
    fill_mode="constant",
cval=0)

f_p = [x for x in os.listdir("/home/martin/Schreibtisch/ds_x/ds_n/p/")]
f_n = [x for x in os.listdir("/home/martin/Schreibtisch/ds_x/ds_n/n/")]

x_pydata = []
y_pydata = []

for img_file in f_p:
    p = load_img(os.path.join("/home/martin/Schreibtisch/ds_x/ds_n/p/", img_file))
    p = p.resize((30,30))
    x = img_to_array(p)
    x = x.reshape((1,) + x.shape)
    x_pydata.append(x[0])
    y_pydata.append(0)

for img_file in f_n:
    p = load_img(os.path.join("/home/martin/Schreibtisch/ds_x/ds_n/n/", img_file))
    p = p.resize((30, 30))

    x = img_to_array(p)
    x = x.reshape((1,) + x.shape)
    x_pydata.append(x[0])
    y_pydata.append(1)

x_data = np.array(x_pydata)
y_data = np.array(y_pydata)

indieces = np.arange(x_data.shape[0])
np.random.shuffle(indieces)
x_data = x_data[indieces]
y_data = y_data[indieces]


train_x = x_data[:(int(0.9*x_data.shape[0]))]
train_y = y_data[:(int(0.9*x_data.shape[0]))]

test_x = x_data[(int(0.9*x_data.shape[0])):]
test_y = y_data[(int(0.9*x_data.shape[0])):]


datagen.fit(train_x)

model = Sequential()

model.add(Convolution2D(32,3,3, input_shape=(30,30,3), activation="relu"))
model.add(MaxPooling2D(pool_size=(2,2), dim_ordering="th"))
model.add(Convolution2D(16,3,3, activation="relu"))
model.add(MaxPooling2D(pool_size=(2,2), dim_ordering="th"))
model.add(Flatten())
model.add(Dense(26, activation="relu"))
model.add(Dense(1, activation="sigmoid"))
model.compile(loss="binary_crossentropy", optimizer="adam", metrics=["accuracy"])

print(model.summary())

hist = model.fit_generator(datagen.flow(
    train_x, train_y, batch_size=1),
    samples_per_epoch=len(train_x),
    validation_data=(test_x, test_y),
    nb_epoch=15,
    verbose=1)

plt.plot(range(len(hist.history["val_acc"])), hist.history["val_acc"])
plt.plot(range(len(hist.history["acc"])), hist.history["acc"])

plt.show()

model.save("model2.ker")

m = model.to_json()
with open("model2.json", "w") as jf:
    jf.write(m)
