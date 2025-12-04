# PDES Simulator Software: DDA-PDES
### A Data-Dependence Analysis Parallel Discrete-Event Simulation Framework for Event-Level Parallelization of General-Purpose DES Models

This repository provides the **DDA-PDES simulation framework**, along with several discrete‑event simulation (DES) models implemented for it.  
For comparison purposes, the repository also includes a **3D torus DES model implemented for the existing ROSS simulator** (ROSS itself is not included).

Automated testing scripts are provided for both the DDA-PDES and ROSS torus models, and each workflow produces CSV-formatted performance results.

---

## 1. Abstract

Utilizing data-dependence analysis (DDA) in parallel discrete-event simulation (PDES) to find event-level parallelism, we present the DDA-PDES framework as an alternative to spatial-decomposition (SD) PDES. DDA-PDES uses a pre-computed Independence Time Limit (ITL) table to efficiently identify events in the pending-event set that are ready for execution, in a shared-memory-parallel simulation engine. Experiments with AMD, Qualcomm, and Intel platforms using several packet-routing network models and a PHOLD benchmark model demonstrate speedup of up to 8.82× and parallel efficiency of up to 0.91. In contrast with DDA-PDES, experiments with similar network models in ROSS demonstrate that SD-PDES cannot speed up the packet-routing models without degradation to routing efficacy. Our results suggest DDA-PDES is an effective method for parallelizing discrete-event simulation models that are computationally intensive, and may be superior to traditional PDES methods for spatially-decomposed models with challenging communication requirements.

---

## 2. Repository Structure

```
/PDDA_simulator_WSC25/              # DDA-PDES framework and DES models
    WSC_network_simulation_results_4096.csv
    torus3d_ROSS_results.csv
/3D_torus_ROSS/                     # DES model written for the ROSS simulator
    /build/                         # Build directory used when compiling with ROSS
```

---

## 3. Building and Running DDA-PDES Models

From within the **PDDA_simulator_WSC25** directory:

```bash
make
python3 WSC_auto_testing_PDDA.py
```

### Output  
Testing results will be generated in:

```
/PDDA_simulator_WSC25/WSC_network_simulation_results_4096.csv
```

---

## 4. Building and Running the ROSS Torus Model

From within the **3D_torus_ROSS** directory:

```bash
cd build
cmake ..
cmake --build .
python3 ../WSC_auto_testing_ROSS.py
```

### Changing Torus Topology (Torus-4 → Torus-1)

Edit **torus_3d_ross_model.c**, line **365**:

```c
g_params.hop_radius = 4;
```

Change it to:

```c
g_params.hop_radius = 1;
```

### Output  
Testing results will appear in:

```
/PDDA_simulator_WSC25/torus3d_ROSS_results.csv
```

