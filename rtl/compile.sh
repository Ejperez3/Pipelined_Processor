vlib work
vlog *.v
vlog *.sv
vsim -c work.hart_tb -do "run -all; -quit" 
