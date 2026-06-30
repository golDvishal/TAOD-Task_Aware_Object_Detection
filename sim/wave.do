#==============================================================================
# wave.do  --  add waveform, run, collect coverage   (called by 'make wave')
#   The Makefile compiles + launches vsim BEFORE running this script, so this
#   file only sets up the Wave window, runs, and saves coverage.
#   Signal paths use the simulator hierarchy (engine internals are flattened
#   into /taod_tb/u_dut), so this file is location-independent.
#==============================================================================

add wave -divider "Clock / Reset"
add wave /taod_tb/clk /taod_tb/rst_n

add wave -divider "Scoring Engine"
add wave /taod_tb/u_dut/eng_start /taod_tb/u_dut/eng_busy \
         /taod_tb/u_dut/eng_done  /taod_tb/u_dut/eng_result_valid
add wave /taod_tb/u_dut/obj_addr  /taod_tb/u_dut/obj_count /taod_tb/u_dut/task_id
add wave /taod_tb/u_dut/res_score /taod_tb/u_dut/res_runner \
         /taod_tb/u_dut/res_class /taod_tb/u_dut/res_idx

add wave -divider "Performance Counters"
add wave /taod_tb/u_dut/perf_comp_last /taod_tb/u_dut/perf_frame_last

add wave -divider "AXI Slave (CPU push / CSR)"
add wave /taod_tb/u_dut/s_awvalid /taod_tb/u_dut/s_awready /taod_tb/u_dut/s_awaddr
add wave /taod_tb/u_dut/s_wvalid  /taod_tb/u_dut/s_wready  /taod_tb/u_dut/s_wdata
add wave /taod_tb/u_dut/s_arvalid /taod_tb/u_dut/s_arready \
         /taod_tb/u_dut/s_rvalid  /taod_tb/u_dut/s_rdata

add wave -divider "AXI Master (DMA)"
add wave /taod_tb/u_dut/m_arvalid /taod_tb/u_dut/m_arready /taod_tb/u_dut/m_araddr \
         /taod_tb/u_dut/m_rvalid  /taod_tb/u_dut/m_rdata
add wave /taod_tb/u_dut/m_awvalid /taod_tb/u_dut/m_awready \
         /taod_tb/u_dut/m_wvalid  /taod_tb/u_dut/m_wdata
add wave /taod_tb/u_dut/dma_busy  /taod_tb/u_dut/dma_done /taod_tb/u_dut/irq

run -all
wave zoom full

coverage report -summary
coverage save taod_cov.ucdb
echo "Waveform ready (Wave window).  Coverage saved -> taod_cov.ucdb"
