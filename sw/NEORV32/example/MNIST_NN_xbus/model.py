# import required modules
import numpy as np
import tensorflow as tf
import struct
from fxpmath import Fxp

# Load MNIST dataset
mnist = tf.keras.datasets.mnist
(train_images, train_labels), (test_images, test_labels) = mnist.load_data()

# Reshape input to flat vector
train_images = np.reshape(train_images, [-1, 28*28])
test_images = np.reshape(test_images, [-1, 28*28])

# Normalize the input image so that each pixel value is between 0 to 1.
train_images = train_images.astype(np.float32) / 255.0
test_images = test_images.astype(np.float32) / 255.0

# network parameters
batch_size = 128
dropout = 0.45

# Define the model architecture
model = tf.keras.Sequential([
    tf.keras.layers.Dense(10, input_dim=28*28, use_bias=False),
    tf.keras.layers.Dropout(dropout),
])

model.compile(optimizer='adam',
            loss=tf.keras.losses.SparseCategoricalCrossentropy(
                from_logits=True),
            metrics=['accuracy'])

model.fit(
    train_images,
    train_labels,
    epochs=20,
    batch_size=batch_size,
    validation_data=(test_images, test_labels)
)

class hex_init_file():
    def __init__(self, basename):
        self.basename = basename
        self.file_idx = 0
        self.addr_idx = 0
        self.file = None
    
    def write(self, data):
        if self.file == None:
            self.file = open(f"{self.basename}_{self.file_idx}.hex", "w")

        self.file.write(f"{data}\n")

        self.addr_idx += 1
        if self.addr_idx > 8191:
            self.addr_idx = 0
            self.file_idx += 1
            self.file.close()
            self.file = None
    
    def close(self):
        self.file.close()
        self.file = None


starting_address = 0x80000400
image = test_images[0]
weights = np.array(model.weights[0])

print("Creating fixed point dmem image and verification file")

# Convert image and weights to fixed point
fxp_ref = Fxp(None, dtype='fxp-s16/7') # 16 bit = 1 sign bit, 8 integer, and 7 fractional
fxp_weights = Fxp(weights, like=fxp_ref)
fxp_image = Fxp(test_images[0], like=fxp_ref)

# Write fixed point model to hex file
file = hex_init_file("neorv32_dmem_image")

# write 0s up until starting address
for i in range(int((starting_address - 0x80000000) / 4)):
    file.write("00000000")

# write image
for i in range(image.shape[0]):
    file.write(f"{fxp_image[i].val & 0xFFFFFFFF:0>8x}")

# write model weights
for i in range(weights.shape[0]):
    for j in range(weights.shape[1]):
        file.write(f"{fxp_weights[i, j].val & 0xFFFFFFFF:0>8x}")

file.close()

# Calculate inference output of fixed point
fxp_outputs = Fxp(np.zeros(10), like=fxp_ref)
raw_outputs = fxp_weights.val.T.dot(fxp_image.val) >> 7
fxp_outputs.set_val(raw_outputs, raw=True)

# Write output to txt file for verification
def int_as_uint(i):
    return struct.unpack('I', struct.pack('i', i))[0]

file = open("verification.out", "w")

for i in range(len(fxp_outputs)):
    file.write(f"{int_as_uint(fxp_outputs[i].val):0>8x}\n")

file.close()
