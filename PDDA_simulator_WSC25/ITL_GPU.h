// GPU-optimized ITL implementation

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/extrema.h>
#include <thrust/execution_policy.h>
#include <thrust/fill.h>
#include <limits>
#include <vector>
#include <chrono>
#include <fstream>
#include <string>

// Constants and typedefs
typedef float dist_t;
const dist_t INF_DIST = std::numeric_limits<dist_t>::max();

// Utility function for CUDA error checking
#define CHECK_CUDA_ERROR(call) \
    do { \
        cudaError_t error = call; \
        if (error != cudaSuccess) { \
            fprintf(stderr, "CUDA error at %s:%d - %s\n", __FILE__, __LINE__, \
                    cudaGetErrorString(error)); \
            exit(EXIT_FAILURE); \
        } \
    } while(0)

// Bit-based connectivity representation instead of sets
// Each vertex's connections are represented as bitsets
// Much more GPU-friendly than set operations
struct BitGraphData {
    thrust::device_vector<unsigned int*> in_edges;   // Incoming edges bitsets
    thrust::device_vector<unsigned int*> out_edges;  // Outgoing edges bitsets
    thrust::device_vector<dist_t*> distances;        // Distances matrix
    thrust::device_vector<dist_t*> itl;              // ITL matrix
    int numVertices;
    int bitsetSize;  // Size of bitset arrays (numVertices / 32 + 1)

    BitGraphData(int nv) : numVertices(nv) {
        bitsetSize = (numVertices + 31) / 32;  // Ceiling of numVertices/32
        
        // Allocate memory for bitsets
        for (int i = 0; i < numVertices; i++) {
            unsigned int* in_bits;
            unsigned int* out_bits;
            dist_t* dist_row;
            dist_t* itl_row;
            
            CHECK_CUDA_ERROR(cudaMalloc(&in_bits, bitsetSize * sizeof(unsigned int)));
            CHECK_CUDA_ERROR(cudaMalloc(&out_bits, bitsetSize * sizeof(unsigned int)));
            CHECK_CUDA_ERROR(cudaMalloc(&dist_row, numVertices * sizeof(dist_t)));
            CHECK_CUDA_ERROR(cudaMalloc(&itl_row, numVertices * sizeof(dist_t)));
            
            in_edges.push_back(in_bits);
            out_edges.push_back(out_bits);
            distances.push_back(dist_row);
            itl.push_back(itl_row);
            
            // Initialize bitsets to 0
            CHECK_CUDA_ERROR(cudaMemset(in_bits, 0, bitsetSize * sizeof(unsigned int)));
            CHECK_CUDA_ERROR(cudaMemset(out_bits, 0, bitsetSize * sizeof(unsigned int)));
        }
    }

    ~BitGraphData() {
        // Free allocated memory
        for (int i = 0; i < numVertices; i++) {
            cudaFree(in_edges[i]);
            cudaFree(out_edges[i]);
            cudaFree(distances[i]);
            cudaFree(itl[i]);
        }
    }
};

// Function to convert CPU graph representation to GPU bit representation
BitGraphData convertToBitGraph(
    const std::vector<std::vector<size_t>>& inEdges,
    const std::vector<std::vector<size_t>>& outEdges,
    const std::vector<std::vector<float>>& shortestPaths,
    int numVertices
) {
    BitGraphData bitGraph(numVertices);
    
    // Convert edge lists to bit representation
    for (int i = 0; i < numVertices; i++) {
        // Convert in-edges
        std::vector<unsigned int> host_in_bits(bitGraph.bitsetSize, 0);
        for (size_t v : inEdges[i]) {
            host_in_bits[v / 32] |= (1u << (v % 32));
        }
        cudaMemcpy(bitGraph.in_edges[i], host_in_bits.data(), 
                  bitGraph.bitsetSize * sizeof(unsigned int), cudaMemcpyHostToDevice);
        
        // Convert out-edges
        std::vector<unsigned int> host_out_bits(bitGraph.bitsetSize, 0);
        for (size_t v : outEdges[i]) {
            host_out_bits[v / 32] |= (1u << (v % 32));
        }
        cudaMemcpy(bitGraph.out_edges[i], host_out_bits.data(), 
                  bitGraph.bitsetSize * sizeof(unsigned int), cudaMemcpyHostToDevice);
        
        // Copy distances
        cudaMemcpy(bitGraph.distances[i], shortestPaths[i].data(), 
                  numVertices * sizeof(dist_t), cudaMemcpyHostToDevice);
    }
    
    return bitGraph;
}

// Kernel to compute reachability matrix (which vertices can reach which others)
__global__ void computeReachabilityKernel(
    dist_t** distances,
    unsigned int** reachability,
    int numVertices,
    int bitsetSize
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numVertices) return;
    
    // For each vertex, determine which other vertices are reachable
    for (int j = 0; j < numVertices; j++) {
        if (distances[i][j] < INF_DIST) {
            // Set bit for reachable vertex
            reachability[i][j / 32] |= (1u << (j % 32));
        }
    }
}

// Kernel to perform bit-based set intersection (replaces std::set_intersection)
__device__ int bitSetIntersection(
    unsigned int* set1,
    unsigned int* set2,
    unsigned int* result,
    int bitsetSize
) {
    int count = 0;
    for (int i = 0; i < bitsetSize; i++) {
        result[i] = set1[i] & set2[i];
        // Count bits set in result (population count)
        count += __popc(result[i]);
    }
    return count;
}

// Kernel to perform bit-based set union (replaces std::set_union)
__device__ void bitSetUnion(
    unsigned int* set1,
    unsigned int* set2,
    unsigned int* result,
    int bitsetSize
) {
    for (int i = 0; i < bitsetSize; i++) {
        result[i] = set1[i] | set2[i];
    }
}

// Kernel for Phase 1 of ITL computation
__global__ void itlPhase1Kernel(
    unsigned int** inEdges,
    unsigned int** outEdges,
    unsigned int** reachability,
    dist_t** distances,
    dist_t** itl,
    int numVertices,
    int bitsetSize
) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= numVertices) return;
    
    // Allocate shared memory for temporary bitsets
    extern __shared__ unsigned int sharedBits[];
    unsigned int* S_k = &sharedBits[0];
    unsigned int* U_Sk = &sharedBits[bitsetSize];
    unsigned int* temp = &sharedBits[2 * bitsetSize];
    unsigned int* X_jk = &sharedBits[3 * bitsetSize];
    
    // Create vertex-k SV set S_k (union of in and out edges)
    bitSetUnion(inEdges[k], outEdges[k], S_k, bitsetSize);
    
    // Initialize U_Sk (vertices that can update S_k)
    for (int i = 0; i < bitsetSize; i++) {
        U_Sk[i] = 0;
    }
    
    // Find vertices that can update S_k
    for (int l = 0; l < numVertices; l++) {
        // Check if outEdges[l] intersects with S_k
        for (int i = 0; i < bitsetSize; i++) {
            temp[i] = outEdges[l][i] & S_k[i];
            if (temp[i] != 0) {
                // l can update S_k, add to U_Sk
                U_Sk[l / 32] |= (1u << (l % 32));
                break;
            }
        }
    }
    
    // Process each potential earlier-event vertex
    for (int j = 0; j < numVertices; j++) {
        // Initialize ITL to infinity
        itl[j][k] = INF_DIST;
        
        // Find vertices reachable from j that can update S_k
        int hasIntersection = 0;
        for (int i = 0; i < bitsetSize; i++) {
            X_jk[i] = reachability[j][i] & U_Sk[i];
            if (X_jk[i] != 0) {
                hasIntersection = 1;
            }
        }
        
        // If intersection exists, find minimum distance
        if (hasIntersection) {
            dist_t minDist = INF_DIST;
            
            // For each bit set in X_jk, check the distance
            for (int i = 0; i < bitsetSize; i++) {
                unsigned int bits = X_jk[i];
                while (bits) {
                    // Find least significant bit
                    int bitPos = __ffs(bits) - 1;
                    int vertexIdx = i * 32 + bitPos;
                    
                    if (vertexIdx < numVertices) {
                        dist_t dist = distances[j][vertexIdx];
                        minDist = min(minDist, dist);
                    }
                    
                    // Clear the processed bit
                    bits &= ~(1u << bitPos);
                }
            }
            
            itl[j][k] = minDist;
        }
    }
}

// Kernel for Phase 2 of ITL computation
__global__ void itlPhase2Kernel(
    unsigned int** reachability,
    dist_t** distances,
    dist_t** itl,
    int numVertices,
    int bitsetSize
) {
    int h = blockIdx.x * blockDim.x + threadIdx.x;
    if (h >= numVertices) return;
    
    // Allocate shared memory for temporary bitsets
    extern __shared__ unsigned int sharedBits[];
    unsigned int* Z_i = &sharedBits[0];
    unsigned int* X_hi = &sharedBits[bitsetSize];
    
    // Process each potential later-event vertex
    for (int i = 0; i < numVertices; i++) {
        // Find vertices that i can affect immediately (ITL[i][l] == 0)
        for (int bitIdx = 0; bitIdx < bitsetSize; bitIdx++) {
            Z_i[bitIdx] = 0;
        }
        
        for (int l = 0; l < numVertices; l++) {
            if (itl[i][l] == 0) {
                Z_i[l / 32] |= (1u << (l % 32));
            }
        }
        
        // Find intersection of reachable vertices and immediately affected vertices
        int hasIntersection = 0;
        for (int bitIdx = 0; bitIdx < bitsetSize; bitIdx++) {
            X_hi[bitIdx] = reachability[h][bitIdx] & Z_i[bitIdx];
            if (X_hi[bitIdx] != 0) {
                hasIntersection = 1;
            }
        }
        
        // If intersection exists, find minimum distance
        if (hasIntersection) {
            dist_t minDist = INF_DIST;
            
            // For each bit set in X_hi, check the distance
            for (int bitIdx = 0; bitIdx < bitsetSize; bitIdx++) {
                unsigned int bits = X_hi[bitIdx];
                while (bits) {
                    // Find least significant bit
                    int bitPos = __ffs(bits) - 1;
                    int vertexIdx = bitIdx * 32 + bitPos;
                    
                    if (vertexIdx < numVertices) {
                        dist_t dist = distances[h][vertexIdx];
                        minDist = min(minDist, dist);
                    }
                    
                    // Clear the processed bit
                    bits &= ~(1u << bitPos);
                }
            }
            
            // Update ITL if phase-two value is smaller
            atomicMin((int*)&itl[h][i], *((int*)&minDist));
        }
    }
}

// Main function for ITL computation on GPU
std::vector<std::vector<float>> MakeITL_GPU(
    const std::vector<std::vector<size_t>>& inEdges,
    const std::vector<std::vector<size_t>>& outEdges,
    const std::vector<std::vector<float>>& shortestPaths,
    int numVertices,
    std::string tableFilename
) {
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Convert graph to bit representation
    BitGraphData bitGraph = convertToBitGraph(inEdges, outEdges, shortestPaths, numVertices);
    
    // Compute reachability matrix
    thrust::device_vector<unsigned int*> reachability;
    int bitsetSize = bitGraph.bitsetSize;
    
    for (int i = 0; i < numVertices; i++) {
        unsigned int* reach_bits;
        CHECK_CUDA_ERROR(cudaMalloc(&reach_bits, bitsetSize * sizeof(unsigned int)));
        CHECK_CUDA_ERROR(cudaMemset(reach_bits, 0, bitsetSize * sizeof(unsigned int)));
        reachability.push_back(reach_bits);
    }
    
    // Kernel configuration
    int blockSize = 256;
    int numBlocks = (numVertices + blockSize - 1) / blockSize;
    
    // Launch reachability kernel
    computeReachabilityKernel<<<numBlocks, blockSize>>>(
        thrust::raw_pointer_cast(bitGraph.distances.data()),
        thrust::raw_pointer_cast(reachability.data()),
        numVertices,
        bitsetSize
    );
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
    
    // Shared memory size for temporary bitsets in Phase 1
    size_t sharedMemSize = 4 * bitsetSize * sizeof(unsigned int);
    
    // Phase 1: Initial ITL computation
    auto p1_start = std::chrono::high_resolution_clock::now();
    
    itlPhase1Kernel<<<numBlocks, blockSize, sharedMemSize>>>(
        thrust::raw_pointer_cast(bitGraph.in_edges.data()),
        thrust::raw_pointer_cast(bitGraph.out_edges.data()),
        thrust::raw_pointer_cast(reachability.data()),
        thrust::raw_pointer_cast(bitGraph.distances.data()),
        thrust::raw_pointer_cast(bitGraph.itl.data()),
        numVertices,
        bitsetSize
    );
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
    
    auto p1_stop = std::chrono::high_resolution_clock::now();
    auto p1_duration = std::chrono::duration_cast<std::chrono::microseconds>(p1_stop - p1_start);
    printf("ITL table phase 1 generation time %lf seconds\n", p1_duration.count() / 1e6);
    
    // Shared memory size for temporary bitsets in Phase 2
    sharedMemSize = 2 * bitsetSize * sizeof(unsigned int);
    
    // Phase 2: Refinement of ITL values
    auto p2_start = std::chrono::high_resolution_clock::now();
    
    itlPhase2Kernel<<<numBlocks, blockSize, sharedMemSize>>>(
        thrust::raw_pointer_cast(reachability.data()),
        thrust::raw_pointer_cast(bitGraph.distances.data()),
        thrust::raw_pointer_cast(bitGraph.itl.data()),
        numVertices,
        bitsetSize
    );
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
    
    auto p2_stop = std::chrono::high_resolution_clock::now();
    auto p2_duration = std::chrono::duration_cast<std::chrono::microseconds>(p2_stop - p2_start);
    printf("ITL table phase 2 generation time %lf seconds\n", p2_duration.count() / 1e6);
    
    // Copy ITL results back to host
    std::vector<std::vector<float>> ITL(numVertices, std::vector<float>(numVertices));
    for (int i = 0; i < numVertices; i++) {
        thrust::device_ptr<dist_t> d_row(bitGraph.itl[i]);
        thrust::copy(d_row, d_row + numVertices, ITL[i].begin());
    }
    
    // Clean up
    for (int i = 0; i < numVertices; i++) {
        cudaFree(reachability[i]);
    }
    
    // Write to CSV
    std::ofstream outFile(tableFilename);
    for (int i = 0; i < numVertices; i++) {
        for (int j = 0; j < numVertices; j++) {
            outFile << (ITL[i][j] == INF_DIST ? "inf" : std::to_string(ITL[i][j]));
            if (j < numVertices - 1) outFile << ",";
        }
        outFile << "\n";
    }
    outFile.close();
    
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
    printf("Total ITL generation time %lf seconds\n", duration.count() / 1e6);
    
    return ITL;
}