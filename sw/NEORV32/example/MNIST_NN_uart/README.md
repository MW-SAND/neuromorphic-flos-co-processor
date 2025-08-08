# MNIST Neural Network inference on NEORV32

- [`main.c`](main.c): Application code for NEORV32
- [`MNIST_NN.ipynb`](MNIST_NN.ipynb): Notebook of creating, training, and quantizing the neural network. This file is not used for building. 
- [`model.py`](model.py): Python file for creating, training, and quantizing the neural network. See `python3 model.py -h` for help. This file is used by the makefile for building and installing the dmem images. 
- [`requirements.txt`](requirements.txt): Required python modules. 
- [`verify_output.py`](verify_output.py): Python file to verify outputs of simulation

## Building

With `make dmem_float`, a python environment gets created and the required packages in `requirements.txt` get installed when the environment does not yet exists. After that the model gets created, trained, and the weights and image gets written to a dmem initialization file `neorv32_dmem_image_float.vhd`. Next to that a verification file `verification_float.out` gets created for verification with [`verify_output.py`](verify_output.py). 

`make install-dmem_float` installs the dmem initialization file `neorv32_dmem_image_float.vhd` to the rtl folder. 

`make dmem_fxp` and `make install-dmem_fxp` do the same, but the weights and image get quantized to fixed point first. 

`make dmem_all` calls under the hood the `dmem_float` and `dmem_fxp` targets. 
