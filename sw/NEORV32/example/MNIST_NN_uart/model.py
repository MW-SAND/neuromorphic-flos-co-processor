import argparse

# parse arguments
parser = argparse.ArgumentParser(description="MNIST NN model creation, training, and weights saving script")
parser.add_argument("-l", "--load-model", nargs="?", const="model.keras", 
                    help="load a model from a file, when no file is specified model.keras is assumed")
parser.add_argument("-s", "--save-model", nargs="?", const="model.keras",
                    help="save the model to a file, when no file is specified model.keras is assumed")
parser.add_argument("-f", "--float-output", nargs="?", const="neorv32_dmem_image_float.vhd",
                    help="save the model weights in floating point to a vhdl initialization file, when no file is specified neorv32_dmem_image_float.vhd is assumed")
parser.add_argument("-vf", "--float-verification", default="verification_float.out",
                    help="verification file for inference result of float model, when no file is specified verification_float.out is assumed")
parser.add_argument("-fxp", "--fxp-output", nargs="?", const="neorv32_dmem_image_fxp.vhd",
                    help="save the model weights in fixed point to a vhdl initialization file, when no file is specified neorv32_dmem_image_fxp.vhd is assumed")
parser.add_argument("-vfxp", "--fxp-verification", default="verification_fxp.out",
                    help="verification file for inference result of fxp model, when no file is specified verification_fxp.out is assumed")
parser.add_argument("-a", "--starting-address", type=lambda x: int(x,0), default=0x80000400,
                    help="address where to start saving the input picture, model, and weights")
parser.add_argument("-i", "--image-index", type=int, default=0,
                    help="what image index of the MNIST database to use for inference")
args = parser.parse_args()

# import required modules
import numpy as np
import tensorflow as tf
import struct
from datetime import datetime
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

if args.load_model:
    model = tf.keras.models.load_model(args.load_model)
else:
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

    if args.save_model:
        model.save(args.save_model)

starting_address = args.starting_address
image = test_images[args.image_index]
weights = np.array(model.weights[0])

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

if args.float_output:
    print("Creating floating point dmem image and verification file")
    
    # Write floating point image and model weights to VHDL initialization file
    def float_as_int(f):
        return struct.unpack('I', struct.pack('f', f))[0]

    file = open(args.float_output, "w")

    # write header
    file.write("-- Generated DMEM image with floating point image and model weights from <MNIST_NN/model.py>\n")
    file.write(f"-- Image at [0x{starting_address:0>8x}, 0x{(starting_address + 784*4 - 1):0>8x}]\n")
    file.write(f"-- Model weights at [0x{(starting_address + 784*4):0>8x}, 0x{(starting_address + 784*4 + 10*784*4 - 1):0>8x}]\n")
    file.write(f"-- Built: {datetime.now().strftime('%d.%m.%Y %H:%M:%S')} (dd.mm.yyyy hh:mm:ss)")
    file.write("\n")
    file.write("package body neorv32_dmem_image is\n")

    for ram_i in range(4):
        file.write("\n")
        file.write(f"constant mem_ram_b{ram_i}_init : mem8_t := (\n")

        # write 0s up until starting address
        for i in range(int((starting_address - 0x80000000) / 4)):
            file.write("x\"00\",\n")

        # write image
        for i in range(image.shape[0]):
            file.write(f"x\"{((float_as_int(image[i]) >> (ram_i * 8)) & 0xFF):0>2x}\",\n")

        # write model weights
        for i in range(weights.shape[0] - 1):
            for j in range(weights.shape[1]):
                file.write(f"x\"{((float_as_int(weights[i, j]) >> (ram_i * 8)) & 0xFF):0>2x}\",\n")
        for j in range(weights.shape[1] - 1):
            file.write(f"x\"{((float_as_int(weights[-1, j]) >> (ram_i * 8)) & 0xFF):0>2x}\",\n")
        file.write(f"x\"{((float_as_int(weights[-1, -1]) >> (ram_i * 8)) & 0xFF):0>2x}\"\n")

        file.write(");\n")

    # write footer
    file.write("\n")
    file.write("end neorv32_dmem_image;\n")

    file.close()

    # Write floating point model to hex file
    file = hex_init_file("neorv32_dmem_image_float")

    # write 0s up until starting address
    for i in range(int((starting_address - 0x80000000) / 4)):
        file.write("00000000")
    
    # write image
    for i in range(image.shape[0]):
        file.write(f"{float_as_int(image[i]):0>8x}")
    
    # write model weights
    for i in range(weights.shape[0]):
        for j in range(weights.shape[1]):
            file.write(f"{float_as_int(weights[i, j]):0>8x}")
    
    file.close()

    # Calculate inference outputs
    outputs = np.zeros(10, np.float32)

    for i in range(10):  # loop over neurons
        for j in range(784):  # loop over input feature = image
            outputs[i] += image[j] * weights[j, i]

    # Write output to txt file for verification
    file = open(args.float_verification, "w")

    file.write("[")
    for i in range(9):
        file.write(f"0x{float_as_int(outputs[i]):0>8x}, ")
    file.write(f"0x{float_as_int(outputs[9]):0>8x}]\n")

    file.close()

if args.fxp_output:
    print("Creating fixed point dmem image and verification file")

    # Convert image and weights to fixed point
    fxp_ref = Fxp(None, dtype='fxp-s16/7') # 16 bit = 1 sign bit, 8 integer, and 7 fractional
    fxp_weights = Fxp(weights, like=fxp_ref)
    fxp_image = Fxp(test_images[0], like=fxp_ref)

    # Write fixed point image and model weights to VHDL initialization file
    file = open(args.fxp_output, "w")

    # write header
    file.write("-- Generated DMEM image with fixed point image and model weights from <MNIST_NN/model.py>\n")
    file.write(f"-- Image at [0x{starting_address:0>8x}, 0x{(starting_address + 784*4 - 1):0>8x}]\n")
    file.write(f"-- Model weights at [0x{(starting_address + 784*4):0>8x}, 0x{(starting_address + 784*4 + 10*784*4 - 1):0>8x}]\n")
    file.write(f"-- Built: {datetime.now().strftime('%d.%m.%Y %H:%M:%S')} (dd.mm.yyyy hh:mm:ss)")
    file.write("\n")
    file.write("package body neorv32_dmem_image is\n")

    for ram_i in range(4):
        file.write("\n")
        file.write(f"constant mem_ram_b{ram_i}_init : mem8_t := (\n")

        # write 0s up until starting address
        for i in range(int((starting_address - 0x80000000) / 4)):
            file.write("x\"00\",\n")

        # write fxp image
        for i in range(fxp_image.shape[0]):
            file.write(f"x\"{((fxp_image[i].val >> (ram_i * 8)) & 0xFF):0>2x}\",\n")

        # write fxp model weights
        for i in range(fxp_weights.shape[0] - 1):
            for j in range(fxp_weights.shape[1]):
                file.write(f"x\"{((fxp_weights[i, j].val >> (ram_i * 8)) & 0xFF):0>2x}\",\n")
        for j in range(fxp_weights.shape[1] - 1):
            file.write(f"x\"{((fxp_weights[-1, j].val >> (ram_i * 8)) & 0xFF):0>2x}\",\n")
        file.write(f"x\"{((fxp_weights[-1, -1].val >> (ram_i * 8)) & 0xFF):0>2x}\"\n")

        file.write(");\n")

    # write footer
    file.write("\n")
    file.write("end neorv32_dmem_image;\n")

    file.close()

    # Write fixed point model to hex file
    file = hex_init_file("neorv32_dmem_image_fxp")

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
    
    file = open(args.fxp_verification, "w")

    file.write("[")
    for i in range(9):
        file.write(f"0x{int_as_uint(fxp_outputs[i].val):0>8x}, ")
    file.write(f"0x{int_as_uint(fxp_outputs[9].val):0>8x}]\n")

    file.close()
