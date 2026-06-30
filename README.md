<div align="center">

# 🚀 TAOD Accelerator
### **Task-Aware Object Detection (Selection & Scoring Accelerator)**

[![SystemVerilog](https://img.shields.io/badge/SystemVerilog-IEEE%201800-blue.svg)]()
[![AXI4](https://img.shields.io/badge/Interface-AXI4-success)]()
[![Platform](https://img.shields.io/badge/Target-VEGA%20RV64IMAFDC-orange)]()
[![FPGA](https://img.shields.io/badge/FPGA-Xilinx%20Genesys--2-red)]()
[![Board](https://img.shields.io/badge/Device-XC7K325T-purple)]()
[![DVCon](https://img.shields.io/badge/DVCon%20India-2026-green)]()
[![License](https://img.shields.io/badge/License-Educational-lightgrey)]()

**DVCon India 2026 Design Contest – Stage 2B**

*A synthesizable hardware accelerator for intelligent object selection from pre-detected objects.*

</div>

---

# 📖 Overview

The **TAOD Accelerator** is a fully synthesizable **SystemVerilog hardware accelerator** that performs **task-aware object selection**.

Instead of performing object detection itself, the accelerator receives **already detected objects** from an upstream detector (such as a **YOLO ensemble**) and determines **which object is most relevant** for a given task.

The design targets the:

- 🖥 **VEGA RV64IMAFDC Processor**
- 🔷 **Genesys-2 FPGA Board**
- ⚡ **Xilinx XC7K325T**
- 🔗 **AXI4 Interface**

---

# 🧠 Scoring Algorithm

For every detected object, the accelerator computes

```text
score = Fconf × Wtask × Tstable × areaTerm
```

where

```text
Fconf = w1 × c1 + w2 × c2
```

| Component | Description |
|------------|-------------|
| **Fconf** | Ensemble confidence fusion |
| **Wtask** | Task/Class weight from on-chip LUT |
| **Tstable** | Temporal stability (EMA per Track ID) |
| **areaTerm** | Bounding-box area contribution |

Finally, the hardware performs an **ArgMax** operation to select:

🏆 Best object

🥈 Runner-up object

---

# 📂 Project Structure

```text
.
├── README.md                 ← Documentation
├── rtl/
│   ├── taod_pkg.sv           ← Parameters & Types (Compile FIRST)
│   ├── taod_engine.sv        ← Scoring Engine
│   ├── taod_axi_slave.sv     ← AXI4-Lite Slave
│   ├── taod_axi_master.sv    ← AXI4 Master DMA
│   └── taod_top.sv           ← Top-Level Integration
│
├── tb/
│   ├── taod_sva.sv           ← Assertions
│   ├── taod_cov.sv           ← Functional Coverage
│   └── taod_tb.sv            ← Testbench
│
├── sim/
│   ├── Makefile
│   └── wave.do
│
└── doc/
    ├── Report
    ├── AXI_Interface_Details
    └── Figures
```

---

# 🛠 Prerequisites

| Tool | Version |
|-------|----------|
| QuestaSim | 2024.1 or newer |
| GNU Make | Latest |

Verify installation

```bash
which vsim
```

Expected output

```text
/path/to/questa/bin/vsim
```

> **Note:** RTL is portable and also works with **Icarus Verilog** and **Cadence Xcelium**. The supplied Makefile targets **QuestaSim**.

---

# ▶ Running Simulation

Go to the simulation directory

```bash
cd sim
make
```

The default target performs

- ✅ RTL Compilation
- ✅ Functional Coverage
- ✅ Code Coverage
- ✅ Simulation
- ✅ Waveform Generation
- ✅ Coverage Reports

---

## Available Commands

| Command | Description |
|----------|-------------|
| `make` | Complete simulation |
| `make wave` | Launch GUI with waveform |
| `make clean` | Remove generated files |
| `make help` | Show all commands |

---

# ⚙ Manual Compilation

```bash
vlib work

vlog -sv +define+USE_COVERGROUPS +cover=bcesfx \
../rtl/taod_pkg.sv \
../rtl/taod_engine.sv \
../rtl/taod_axi_slave.sv \
../rtl/taod_axi_master.sv \
../rtl/taod_top.sv \
../tb/taod_sva.sv \
../tb/taod_cov.sv \
../tb/taod_tb.sv

vsim -c -coverage -voptargs=+acc work.taod_tb \
-do "run -all; quit"
```

> ⚠ **Compile Order Matters**
>
> `taod_pkg.sv` **must always be compiled first**.

---

# ✅ Expected Output

Successful execution prints

```text
RESULT: ALL CHECKS PASSED
(golden == RTL:
8 PUSH Frames
2 DMA Frames
SVA Clean)
```

This confirms

- ✅ Golden Model Match
- ✅ 10/10 Assertions Passed
- ✅ Functional Coverage Completed

---

# 📊 Coverage Results

| Metric | Result |
|----------|---------|
| Assertions | **100%** |
| FSM Coverage | **100%** |
| Statement Coverage | **~97%** |
| Functional Coverage | **~91%** |

---

# ⚡ Performance

Measured directly from on-chip performance counters.

```text
Latency = 6 × N + 2 cycles
```

Example

| Objects | Cycles |
|-----------|---------|
| 10 | 62 cycles |

At **50 MHz**

```text
≈ 1.24 μs
```

---

# 📄 Generated Files

After simulation

| File | Description |
|--------|-------------|
| `sim_run.log` | Complete transcript |
| `coverage_summary.txt` | Coverage summary |
| `coverage_detailed.txt` | Detailed coverage |
| `taod_cov.ucdb` | Questa Coverage Database |
| `vsim.wlf` | Waveform |
| `work/` | Compiled library |

Generate HTML coverage

```bash
vcover report \
-html \
-htmldir cov_html \
-details sim/taod_cov.ucdb
```

---

# 🌊 Waveform Viewing

GUI

```bash
make wave
```

Headless

```bash
vsim -view vsim.wlf
```

Recommended zoom

```text
13300 ns → 14400 ns
```

CPU Push Frame

```text
17600 ns → 19900 ns
```

DMA Frames

---

# 🧩 AXI Register Map

Base Address

```text
0x2006_0000
```

| Offset | Register | Description |
|----------|----------|-------------|
| `0x000` | CTRL | Start / Clear / DMA Start |
| `0x008` | TASK | Task ID |
| `0x010` | OBJCNT | Number of Objects |
| `0x018` | FUSE | Fusion Weights |
| `0x020` | INVFA | Inverse Frame Area |
| `0x028` | ALPHA | EMA Coefficient |
| `0x0F8` | VERSION | 0x7A0D0001 |
| `0x100` | STATUS | Busy / Done |
| `0x108` | RESULT | Winning Score |
| `0x110` | RUNNER | Runner-up Score |
| `0x118` | BBOX | Winning Bounding Box |
| `0x120` | PERF | Compute Cycles |
| `0x128` | PERF | Total Frame Cycles |

Complete AXI documentation is available in **`doc/`**.

---

# 🎯 Design Highlights

✨ Fully Synthesizable RTL

⚡ AXI4-Lite Slave Interface

🚀 AXI4 Master DMA Engine

📊 On-chip Performance Counters

🧠 Task-Aware Scoring Engine

📈 Temporal Stability using EMA

🎯 ArgMax Selection Hardware

🛡 SystemVerilog Assertions

📈 Functional & Code Coverage

🏆 Golden Model Verification

---

# 📝 Notes

- Object detection (YOLO or any detector) executes in **software**.
- The accelerator only performs **task-aware selection and scoring**.
- Board integration on **VEGA + Genesys-2** is planned for **Stage 3**.
- Standalone synthesis reports high I/O utilization because AXI interfaces are exposed as FPGA pins. During SoC integration (or out-of-context synthesis), resource utilization becomes representative.

---

<div align="center">

## ⭐ TAOD Accelerator

**Task-Aware Object Selection Hardware Accelerator**

**DVCon India 2026 Design Contest**

Built with ❤️ using **SystemVerilog**, **AXI4**, and **FPGA Hardware Design**

</div>