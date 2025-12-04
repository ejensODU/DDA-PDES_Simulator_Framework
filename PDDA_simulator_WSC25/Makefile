CXX = g++
CXXFLAGS = -std=c++20 -fopenmp -O3

# Enable CUDA support if specified (make USE_CUDA=1)
ifdef USE_CUDA
    # CUDA compiler and flags
    NVCC = nvcc
    NVCCFLAGS = -O3 -std=c++20 -arch=sm_60 -Xcompiler -fopenmp
    CXXFLAGS += -DUSE_CUDA
    CUDA_LDFLAGS = -lcudart -lcuda
    # Add CUDA files
    CUDA_OBJECTS = FW_GPU.o ITL_GPU.o
else
    CUDA_OBJECTS =
endif

# Enable optimized CPU implementation if specified (make USE_OPTIMIZED_CPU=1, enabled by default)
ifndef USE_OPTIMIZED_CPU
    USE_OPTIMIZED_CPU = 1
endif

ifeq ($(USE_OPTIMIZED_CPU), 1)
    CXXFLAGS += -DUSE_OPTIMIZED_CPU
endif

VPATH = PHOLD Ring_1D INoC Grid_VN2D Grid_VN3D Torus_3D

# Base objects
BASE_OBJECTS = Dist.o Vertex.o PDDA_SimModel.o PDDA_SimExec.o PDDA_SV.o

# Grid objects
PHOLD_OBJECTS = PHOLD_Arrive.o PHOLD_Model.o
RING1D_OBJECTS = Ring_1D_Packet.o Ring_1D_Arrive.o Ring_1D_Depart.o Ring_1D.o
INoC_OBJECTS = Irregular_NoC_Packet.o Irregular_NoC_NeighborInfo.o Irregular_NoC_Arrive.o Irregular_NoC_Depart.o Irregular_NoC.o
VN2D_OBJECTS = Grid_VN2D_Packet.o Grid_VN2D_NeighborInfo.o Grid_VN2D_Arrive.o Grid_VN2D_Depart.o Grid_VN2D.o
VN3D_OBJECTS = Grid_VN3D_Packet.o Grid_VN3D_NeighborInfo.o Grid_VN3D_Arrive.o Grid_VN3D_Depart.o Grid_VN3D.o
TORUS3D_OBJECTS = Torus_3D_Packet.o Torus_3D_NeighborInfo.o Torus_3D_Arrive.o Torus_3D_Depart.o Torus_3D.o

OBJECTS = $(BASE_OBJECTS) $(PHOLD_OBJECTS) $(RING1D_OBJECTS) $(INoC_OBJECTS) $(VN2D_OBJECTS) $(VN3D_OBJECTS) $(TORUS3D_OBJECTS) $(CUDA_OBJECTS)

all: PDDA_Sim

PDDA_Sim: PDDA_Sim.cpp $(OBJECTS)
ifdef USE_CUDA
	$(CXX) $(CXXFLAGS) -o $@ $^ $(CUDA_LDFLAGS)
else
	$(CXX) $(CXXFLAGS) -o $@ $^
endif

# CUDA compilation rules (only used if USE_CUDA=1)
ifdef USE_CUDA
FW_GPU.o: FW_GPU.cu FW_GPU.h
	$(NVCC) $(NVCCFLAGS) -c $< -o $@

ITL_GPU.o: ITL_GPU.cu ITL_GPU.h
	$(NVCC) $(NVCCFLAGS) -c $< -o $@
endif

# Base compilation rules
Dist.o: Dist.cpp Dist.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Vertex.o: Vertex.cpp Vertex.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

PDDA_SimModel.o: PDDA_SimModel.cpp PDDA_SimModel.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

PDDA_SimExec.o: PDDA_SimExec.cpp PDDA_SimExec.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

PDDA_SV.o: PDDA_SV.cpp PDDA_SV.h PDDA_SV.tpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


# PHOLD compilation rules
PHOLD_Arrive.o: PHOLD/PHOLD_Arrive.cpp PHOLD/PHOLD_Arrive.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

PHOLD_Model.o: PHOLD/PHOLD_Model.cpp PHOLD/PHOLD_Model.h
	$(CXX) $(CXXFLAGS) -c $< -o $@


# 1D Ring compilation rules
Ring_1D_Packet.o: Ring_1D/Ring_1D_Packet.cpp Ring_1D/Ring_1D_Packet.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Ring_1D_Arrive.o: Ring_1D/Ring_1D_Arrive.cpp Ring_1D/Ring_1D_Arrive.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Ring_1D_Depart.o: Ring_1D/Ring_1D_Depart.cpp Ring_1D/Ring_1D_Depart.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Ring_1D.o: Ring_1D/Ring_1D.cpp Ring_1D/Ring_1D.h
	$(CXX) $(CXXFLAGS) -c $< -o $@


# Irregular NoC Grid compilation rules
Irregular_NoC_Packet.o: Irregular_NoC/Irregular_NoC_Packet.cpp Irregular_NoC/Irregular_NoC_Packet.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Irregular_NoC_NeighborInfo.o: Irregular_NoC/Irregular_NoC_NeighborInfo.cpp Irregular_NoC/Irregular_NoC_NeighborInfo.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Irregular_NoC_Arrive.o: Irregular_NoC/Irregular_NoC_Arrive.cpp Irregular_NoC/Irregular_NoC_Arrive.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Irregular_NoC_Depart.o: Irregular_NoC/Irregular_NoC_Depart.cpp Irregular_NoC/Irregular_NoC_Depart.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Irregular_NoC.o: Irregular_NoC/Irregular_NoC.cpp Irregular_NoC/Irregular_NoC.h
	$(CXX) $(CXXFLAGS) -c $< -o $@


# 2D Grid compilation rules
Grid_VN2D_Packet.o: Grid_VN2D/Grid_VN2D_Packet.cpp Grid_VN2D/Grid_VN2D_Packet.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Grid_VN2D_NeighborInfo.o: Grid_VN2D/Grid_VN2D_NeighborInfo.cpp Grid_VN2D/Grid_VN2D_NeighborInfo.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Grid_VN2D_Arrive.o: Grid_VN2D/Grid_VN2D_Arrive.cpp Grid_VN2D/Grid_VN2D_Arrive.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Grid_VN2D_Depart.o: Grid_VN2D/Grid_VN2D_Depart.cpp Grid_VN2D/Grid_VN2D_Depart.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Grid_VN2D.o: Grid_VN2D/Grid_VN2D.cpp Grid_VN2D/Grid_VN2D.h
	$(CXX) $(CXXFLAGS) -c $< -o $@


# 3D Grid compilation rules
Grid_VN3D_Packet.o: Grid_VN3D/Grid_VN3D_Packet.cpp Grid_VN3D/Grid_VN3D_Packet.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Grid_VN3D_NeighborInfo.o: Grid_VN3D/Grid_VN3D_NeighborInfo.cpp Grid_VN3D/Grid_VN3D_NeighborInfo.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Grid_VN3D_Arrive.o: Grid_VN3D/Grid_VN3D_Arrive.cpp Grid_VN3D/Grid_VN3D_Arrive.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Grid_VN3D_Depart.o: Grid_VN3D/Grid_VN3D_Depart.cpp Grid_VN3D/Grid_VN3D_Depart.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Grid_VN3D.o: Grid_VN3D/Grid_VN3D.cpp Grid_VN3D/Grid_VN3D.h
	$(CXX) $(CXXFLAGS) -c $< -o $@


# 3D Torus compilation rules
Torus_3D_Packet.o: Torus_3D/Torus_3D_Packet.cpp Torus_3D/Torus_3D_Packet.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Torus_3D_NeighborInfo.o: Torus_3D/Torus_3D_NeighborInfo.cpp Torus_3D/Torus_3D_NeighborInfo.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Torus_3D_Arrive.o: Torus_3D/Torus_3D_Arrive.cpp Torus_3D/Torus_3D_Arrive.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Torus_3D_Depart.o: Torus_3D/Torus_3D_Depart.cpp Torus_3D/Torus_3D_Depart.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

Torus_3D.o: Torus_3D/Torus_3D.cpp Torus_3D/Torus_3D.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f PDDA_Sim $(OBJECTS)

help:
	@echo "Makefile options:"
	@echo "  make               - Build with default options (CPU only)"
	@echo "  make USE_CUDA=1    - Build with CUDA support"
	@echo "  make USE_OPTIMIZED_CPU=0 - Disable optimized CPU implementation"
	@echo "  make clean         - Remove compiled files"
	@echo "  make help          - Display this help message"

.PHONY: all clean help