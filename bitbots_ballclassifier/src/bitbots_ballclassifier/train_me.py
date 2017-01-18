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
        #rescale=1./255,
        #shear_range=0.1,
        zoom_range=0.1,
        horizontal_flip=True,
    fill_mode="reflect",
cval=0)


pos = "/home/martin/Schreibtisch/ds_x/ds_pk2/"
neg = "/home/martin/Schreibtisch/ds_x/ds_nk2/"
f_p = [x for x in os.listdir(pos)]
f_n = [x for x in os.listdir(neg)]

x_pydata = []
y_pydata = []

for img_file in f_p:
    p = load_img(os.path.join(pos, img_file))
    p = p.resize((30,30))
    x = img_to_array(p)
    x /= 255
    x = x.reshape((1,) + x.shape)
    x_pydata.append(x[0])
    y_pydata.append(1)

for img_file in f_n:
    p = load_img(os.path.join(neg, img_file))
    p = p.resize((30, 30))

    x = img_to_array(p)
    x /= 255.0
    x = x.reshape((1,) + x.shape)
    x_pydata.append(x[0])
    y_pydata.append(0)

x_data = np.array(x_pydata)
y_data = np.array(y_pydata)

indieces = np.arange(x_data.shape[0])
np.random.shuffle(indieces)
x_data = x_data[indieces]
y_data = y_data[indieces]


train_x = x_data[:(int(0.9*x_data.shape[0]))]
train_y = y_data[:(int(0.9*x_data.shape[0]))]

val_x = x_data[(int(0.9 * x_data.shape[0])):]
val_y = y_data[(int(0.9 * x_data.shape[0])):]


datagen.fit(train_x)

model = Sequential()

model.add(Convolution2D(16,3,3, input_shape=(30,30,3), activation="relu"))
model.add(MaxPooling2D(pool_size=(2,2), dim_ordering="th"))
model.add(Convolution2D(8,3,3, activation="relu"))
model.add(MaxPooling2D(pool_size=(2,2), dim_ordering="th"))
#model.add(Convolution2D(16,3,3, activation="relu"))
#model.add(MaxPooling2D(pool_size=(2,2), dim_ordering="th"))
model.add(Flatten())
model.add(Dense(7, activation="relu"))
model.add(Dense(1, activation="sigmoid"))
model.compile(loss="binary_crossentropy", optimizer="adam", metrics=["accuracy"])

print(model.summary())

cb = []
cb.append(keras.callbacks.TensorBoard(log_dir='./logs', histogram_freq=1, write_graph=True, write_images=False))

hist = model.fit_generator(datagen.flow(
    train_x, train_y, batch_size=32),
    samples_per_epoch=len(train_x),
    validation_data=(val_x, val_y),
    nb_epoch=5,
    verbose=1,
    callbacks=cb)

#hist = model.fit(train_x, train_y, validation_data=(val_x, val_y), callbacks=cb, nb_epoch=12, verbose=1)
plt.plot(range(len(hist.history["val_acc"])), hist.history["val_acc"])
plt.plot(range(len(hist.history["acc"])), hist.history["acc"])

plt.show()

model.save("model3.ker")

m = model.to_json()
with open("model3.json", "w") as jf:
    jf.write(m)
