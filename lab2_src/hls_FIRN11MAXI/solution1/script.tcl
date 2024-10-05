############################################################
## This file is generated automatically by Vivado HLS.
## Please DO NOT edit it.
## Copyright (C) 1986-2019 Xilinx, Inc. All Rights Reserved.
############################################################
open_project hls_FIRN11MAXI
set_top fir_n11_maxi
add_files hls_FIRN11MAXI/FIR.cpp
add_files hls_FIRN11MAXI/FIR.h
add_files -tb hls_FIRN11MAXI/FIRTester.cpp
open_solution "solution1"
set_part {xc7z020clg484-1}
create_clock -period 5 -name default
config_export -format ip_catalog -rtl verilog
#source "./hls_FIRN11MAXI/solution1/directives.tcl"
csim_design
csynth_design
cosim_design
export_design -rtl verilog -format ip_catalog
