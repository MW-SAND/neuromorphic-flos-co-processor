#!/bin/bash

# Create both application and data hex files

cd ../../image-gen
rm -f *.hex
python3 row_interleaved_hex_file_generator.py ../app/VGG16/vgg16_block2_conv1_constants_rlc.h 128 8 0 2048

cd ../app/VGG16/
python3 compile_nasm_to_header.py conv_acc_program.nasm acc_program.h
python3 hfai_to_cheader.py

make clean
make all

mv neorv32_application_image.vhd ../../image-gen
cd ../../image-gen

python3 convert_vhd_to_hex.py neorv32_application_image.vhd
python3 hex_file_splitter_noninterleaved.py neorv32_application_image.hex 8 1024
