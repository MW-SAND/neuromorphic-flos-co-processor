echo "Removing old files"
mkdir -p genus_workspace
rm -r genus_workspace/*
rm -r ._*

echo "Preparing file list"
SOC_LIST=../rtl/file_list_soc_synthesis.f

cp $SOC_LIST file_list.f

sed '4,$d; s/NEORV32_RTL_PATH_PLACEHOLDER/..\/rtl/g' file_list.f > file_list_synthesis_verilog.f
sed '1,3d; s/NEORV32_RTL_PATH_PLACEHOLDER/..\/rtl/g' file_list.f > file_list_synthesis_vhdl.f

module load cadence/genus/23.33
cd genus_workspace
genus -f ../genus_script.tcl
module unload cadence/genus/23.33
