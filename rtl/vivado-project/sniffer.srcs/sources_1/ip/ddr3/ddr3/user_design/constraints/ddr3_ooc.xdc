###################################################################################################
## This constraints file contains default clock frequencies to be used during creation of a 
## Synthesis Design Checkpoint (DCP). For best results the frequencies should be modified 
## to match the target frequencies. 
## This constraints file is not used in top-down/global synthesis (not the default flow of Vivado).
###################################################################################################


##################################################################################################
## 
##  Xilinx, Inc. 2010            www.xilinx.com 
##  Tue Dec  7 22:57:52 2021

##  Generated by MIG Version 4.2
##  
##################################################################################################
##  File name :       ddr3.xdc
##  Details :     Constraints file
##                    FPGA Family:       KINTEX7
##                    FPGA Part:         XC7K70T-FBG484
##                    Speedgrade:        -2
##                    Design Entry:      VERILOG
##                    Frequency:         333.32999999999998 MHz
##                    Time Period:       3000 ps
##################################################################################################

##################################################################################################
## Controller 0
## Memory Device: DDR3_SDRAM->SODIMMs->Kingston_KVR16LS11S6_2
## Data Width: 64
## Time Period: 3000
## Data Mask: 1
##################################################################################################

create_clock -period 3 [get_ports sys_clk_i]
          
create_clock -period 5 [get_ports clk_ref_i]
          