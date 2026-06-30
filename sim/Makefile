#==============================================================================
# Makefile  --  TAOD accelerator simulation  (QuestaSim)
#
#   Layout:  rtl/ (5 files)   tb/ (3 files)   sim/ (this Makefile + wave.do)
#   Run from inside  sim/
#
#   Just type:   make            (runs everything, headless: log+coverage+wave file)
#   Other:       make wave       (live GUI waveform -- needs display / ssh -X)
#                make clean      (remove generated files)
#                make help       (show this)
#==============================================================================

RTL := ../rtl
TB  := ../tb

# COMPILE ORDER --> taod_pkg MUST be first
SRCS := \
    $(RTL)/taod_pkg.sv \
    $(RTL)/taod_engine.sv \
    $(RTL)/taod_axi_slave.sv \
    $(RTL)/taod_axi_master.sv \
    $(RTL)/taod_top.sv \
    $(TB)/taod_sva.sv \
    $(TB)/taod_cov.sv \
    $(TB)/taod_tb.sv

TOP := taod_tb
COV := -sv +define+USE_COVERGROUPS +cover=bcesfx
ACC := -voptargs=+acc

.DEFAULT_GOAL := sim
.PHONY: sim wave clean help

#  DEFAULT: bare 'make' -> compile + run + coverage + dump waveform (headless) 

sim:
	-vlib work
	vlog $(COV) $(SRCS)
	vsim -c -coverage $(ACC) work.$(TOP) -do "log -r /*; run -all; coverage report -summary; coverage report -summary -file coverage_summary.txt; coverage report -details -file coverage_detailed.txt; coverage save taod_cov.ucdb; quit" -l sim_run.log
	@echo ""
	@echo "  DONE (headless).  Outputs in sim/ :"
	@echo "    sim_run.log            full transcript (look for ALL CHECKS PASSED)"
	@echo "    coverage_summary.txt   coverage summary"
	@echo "    coverage_detailed.txt  detailed coverage"
	@echo "    taod_cov.ucdb          coverage database"
	@echo "    vsim.wlf               waveform  (view with: make wave)"

#  live GUI waveform (needs a display; over SSH use: ssh -X) 

wave:
	-vlib work
	vlog $(COV) $(SRCS)
	vsim -coverage $(ACC) work.$(TOP) -do "do wave.do"

clean:
	-vdel -all
	-rm -rf work transcript *.wlf sim_run.log coverage_summary.txt coverage_detailed.txt taod_cov.ucdb
	@echo "cleaned."

help:
	@echo "=================================================================="
	@echo "  TAOD accelerator simulation  (QuestaSim)"
	@echo "=================================================================="
	@echo "  make         run EVERYTHING headless (log + coverage + waveform file)"
	@echo "  make wave    open the live waveform GUI (needs display / ssh -X)"
	@echo "  make clean   remove generated files"
	@echo "  make help    show this message"
	@echo "=================================================================="