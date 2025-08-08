""" 
This script has been designed and implemented by the author
"""

import tensorflow as tf
from tensorflow.keras.applications import VGG16
from tensorflow.keras.models import Model, Sequential
from tensorflow.keras.layers import Dense, Flatten, Dropout, Conv2D
from tensorflow.keras import optimizers
from tensorflow.keras.datasets import cifar10
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping, LearningRateScheduler
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.utils import plot_model
import numpy as np
from tensorflow.keras import backend as K
from tensorflow.keras import regularizers

# Load CIFAR-10 dataset
(x_train, y_train), (x_test, y_test) = cifar10.load_data()
x_train, x_test = np.floor(
    x_train / 36).astype('float32'), np.floor(x_test / 36).astype('float32')  # Normalize data
y_train, y_test = to_categorical(y_train, 10), to_categorical(y_test, 10)

x_val = x_train[40000:]
y_val = y_train[40000:]

x_train = x_train[:40000]
y_train = y_train[:40000]


# Load pre-trained VGG16 model without top layers
base_model = VGG16(weights='imagenet', include_top=False,
                   classes=10, input_shape=(32, 32, 3))

# Add custom layers on top
model = Sequential()

for layer in base_model.layers:
    model.add(layer)

model.add(Flatten())
model.add(Dense(512, activation='relu'))
model.add(Dropout(0.55))
model.add(Dense(256, activation='relu'))
model.add(Dropout(0.55))
model.add(Dense(10, activation='softmax', name='predictions'))

model.summary()

# Save and display the model architecture
plot_model(model, to_file='model_architecture.png',
           show_shapes=True, show_layer_names=True)

# Compile model
mc = ModelCheckpoint('VGG16_mc_SGD_D55.model',
                     monitor='val_accuracy', save_best_only=True, mode='max')

sgd = optimizers.SGD(learning_rate=0.001, momentum=0.9)
model.compile(optimizer=sgd, loss='categorical_crossentropy',
              metrics=['accuracy'])

# Determine the learning rate
def lr_scheduler(epoch):
    return 0.001 * (0.1 ** (epoch // 20))

reduce_lr = LearningRateScheduler(lr_scheduler)

# construct the training image generator for data augmentation
aug = ImageDataGenerator(
    rotation_range=20,
    zoom_range=0.15,
    width_shift_range=0.2,
    height_shift_range=0.2,
    shear_range=0.15,
    horizontal_flip=True,
    fill_mode="nearest")

batch_size = 128
epochs = 30

# Train the model
history = model.fit(
    aug.flow(x_train, y_train, batch_size=batch_size),
    validation_data=(x_val, y_val),
    steps_per_epoch=len(x_train) // batch_size,
    epochs=epochs,
    callbacks=[reduce_lr, mc])


model.save("VGG16_SGD_D55.model")
model.load_weights('VGG16_mc_SGD_D55.model')

# Evaluate model
val_loss, val_accuracy = model.evaluate(x_val, y_val)
print('Validation loss: {}\nValidation accuracy: {}'.format(val_loss, val_accuracy))

loss, accuracy = model.evaluate(x_test, y_test)
print(f'Validation loss: {loss:.2f}\nTest Accuracy: {accuracy:.2f}')
