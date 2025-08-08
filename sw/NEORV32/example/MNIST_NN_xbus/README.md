# MNIST Neural Network inference on NEORV32

- [`main.c`](main.c): Application code for NEORV32
- [`model.py`](model.py): Python file for creating, training, and quantizing the neural network. Generates the neorv32_dmem_image_*.hex files. 
- [`requirements.txt`](requirements.txt): Required python modules. 

## Building

`make dmem` creates the neorv32_dmem_image_*.hex files by creating a virtual environment with required python packages if its not there and calling model.py.

`install dmem` copies/installs the neorv32_dmem_image_*.hex files to rtl/core folder.

`make install-hex` creates and copies the imem image in hex format to the rtl/core folder. 
