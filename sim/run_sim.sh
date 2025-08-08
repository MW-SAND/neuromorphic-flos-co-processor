sh clean_sim.sh

echo "Preparing file list"
SOC_LIST=../rtl/file_list_soc_simulation.f

cp $SOC_LIST file_list_sim.f

sed -i 's/NEORV32_RTL_PATH_PLACEHOLDER/..\/rtl/g; $a\
tb_neo_acc.v' file_list_sim.f

echo "Loading Module Xcelium"
echo "Module is:: $MODULEPATH"
#export MODULEPATH=/path/to/modulefiles:$MODULEPATH
module load cadence/xcelium/23.03.007

echo "Compiling & executing files"
xrun -64bit -messages -access +rwc -work neorv32 -f file_list_sim.f -top tb_neo_acc -v93 -timescale 1ps/1ps +define+VIRAGE_FAST_VERILOG -input debug_simt.tcl +define+INITIALIZE_MEM -gui

#work: Name of the working lib that will be used instead of default worklib nested in xcelium.d
#f: file list of all the rtl codes in order of execution
#top: name of top module . with the name of the working lib
#-v93: support features of VHDL 93
#timescale: timescale of all the verilog modules in the design

echo "UnLoading Module Xcelium"
module unload cadence/xcelium/23.03.00

#Extra lines for compiling and running
#chmod +x run_xcelium.sh
#./run_xcelium.sh

# rm -rf xcelium.d file_list.f *.out xrun.*
