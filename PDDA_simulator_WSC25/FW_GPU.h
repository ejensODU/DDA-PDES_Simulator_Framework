#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <chrono>
#include <vector>
#include <limits>
#include <iostream>

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

// Structure to represent an edge in the graph
struct Edge {
    size_t sourceId;
    size_t targetId;
    dist_t weight;
    
    Edge(size_t src, size_t tgt, dist_t w) : sourceId(src), targetId(tgt), weight(w) {}
};

// Kernel for initializing distance matrix
__global__ void initializeDistanceMatrix(dist_t* d_dist, int numVertices) {
    int i = blockIdx.y * blockDim.y + threadIdx.y;
    int j = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (i < numVertices && j < numVertices) {
        int idx = i * numVertices + j;
        if (i == j) {
            d_dist[idx] = 0.0f;  // Distance to self is 0
        } else {
            d_dist[idx] = INF_DIST;  // Initialize all other distances to infinity
        }
    }
}

// Kernel for populating initial edge distances
__global__ void populateEdges(dist_t* d_dist, const Edge* d_edges, int numEdges, int numVertices) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx < numEdges) {
        const Edge& edge = d_edges[idx];
        if (edge.sourceId != edge.targetId) {  // Skip self-loops
            int distIdx = edge.sourceId * numVertices + edge.targetId;
            d_dist[distIdx] = edge.weight;
        }
    }
}

// Kernel for Floyd-Warshall relaxation step (optimized)
__global__ void floydWarshallKernel(dist_t* d_dist, int k, int numVertices) {
    // Declare shared memory for the k-th row and column
    extern __shared__ dist_t sharedMem[];
    dist_t* kRow = &sharedMem[0];                  // k-th row
    dist_t* kCol = &sharedMem[blockDim.x];         // k-th column
    
    int i = blockIdx.y * blockDim.y + threadIdx.y;
    int j = blockIdx.x * blockDim.x + threadIdx.x;
    
    // Load k-th row and column elements into shared memory
    if (threadIdx.y == 0 && j < numVertices) {
        kRow[threadIdx.x] = d_dist[k * numVertices + j];
    }
    if (threadIdx.x == 0 && i < numVertices) {
        kCol[threadIdx.y] = d_dist[i * numVertices + k];
    }
    
    // Ensure shared memory is loaded
    __syncthreads();
    
    if (i < numVertices && j < numVertices) {
        int idx = i * numVertices + j;
        dist_t i_to_k = kCol[threadIdx.y];
        dist_t k_to_j = kRow[threadIdx.x];
        
        // Only process if valid path through k exists
        if (i_to_k != INF_DIST && k_to_j != INF_DIST) {
            dist_t newDist = i_to_k + k_to_j;
            dist_t oldDist = d_dist[idx];
            
            if (newDist < oldDist) {
                d_dist[idx] = newDist;
            }
        }
    }
}

// Tiled version of Floyd-Warshall for very large graphs
__global__ void floydWarshallTiledKernel(dist_t* d_dist, int k, int numVertices, int tileSize) {
    extern __shared__ dist_t tileMem[];
    
    // Calculate tile indices
    int tileRow = blockIdx.y;
    int tileCol = blockIdx.x;
    
    // Calculate matrix indices for this thread
    int i = tileRow * tileSize + threadIdx.y;
    int j = tileCol * tileSize + threadIdx.x;
    
    // Load tile into shared memory
    int tileIdx = threadIdx.y * tileSize + threadIdx.x;
    if (i < numVertices && j < numVertices) {
        tileMem[tileIdx] = d_dist[i * numVertices + j];
    } else {
        tileMem[tileIdx] = INF_DIST;
    }
    
    __syncthreads();
    
    // The k-th row and column may be in different tiles
    dist_t i_to_k = (i < numVertices) ? d_dist[i * numVertices + k] : INF_DIST;
    dist_t k_to_j = (j < numVertices) ? d_dist[k * numVertices + j] : INF_DIST;
    
    // Perform the Floyd-Warshall relaxation step
    if (i < numVertices && j < numVertices) {
        if (i_to_k != INF_DIST && k_to_j != INF_DIST) {
            dist_t newDist = i_to_k + k_to_j;
            if (newDist < tileMem[tileIdx]) {
                tileMem[tileIdx] = newDist;
            }
        }
    }
    
    __syncthreads();
    
    // Write updated tile back to global memory
    if (i < numVertices && j < numVertices) {
        d_dist[i * numVertices + j] = tileMem[tileIdx];
    }
}

// Optimized version that processes multiple k values in parallel for small graphs
__global__ void floydWarshallBatchKernel(dist_t* d_dist, int startK, int batchSize, int numVertices) {
    int i = blockIdx.y * blockDim.y + threadIdx.y;
    int j = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (i < numVertices && j < numVertices) {
        int idx = i * numVertices + j;
        dist_t current = d_dist[idx];
        
        // Process a batch of k values
        for (int k_offset = 0; k_offset < batchSize; k_offset++) {
            int k = startK + k_offset;
            if (k >= numVertices) break;
            
            dist_t i_to_k = d_dist[i * numVertices + k];
            dist_t k_to_j = d_dist[k * numVertices + j];
            
            if (i_to_k != INF_DIST && k_to_j != INF_DIST) {
                dist_t newDist = i_to_k + k_to_j;
                if (newDist < current) {
                    current = newDist;
                }
            }
        }
        
        d_dist[idx] = current;
    }
}

// Main function for Floyd-Warshall computation on GPU
std::vector<std::vector<float>> FloydWarshall_GPU(
    const std::vector<std::vector<Edge>>& edges,
    int numVertices
) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Flatten edges into a single vector for GPU
    std::vector<Edge> flattenedEdges;
    for (size_t i = 0; i < edges.size(); i++) {
        for (const auto& edge : edges[i]) {
            flattenedEdges.push_back(edge);
        }
    }
    int numEdges = flattenedEdges.size();
    
    // Allocate device memory for distance matrix and edges
    dist_t* d_dist;
    Edge* d_edges;
    size_t distSize = numVertices * numVertices * sizeof(dist_t);
    size_t edgesSize = numEdges * sizeof(Edge);
    
    CHECK_CUDA_ERROR(cudaMalloc(&d_dist, distSize));
    CHECK_CUDA_ERROR(cudaMalloc(&d_edges, edgesSize));
    
    // Copy edges to device
    CHECK_CUDA_ERROR(cudaMemcpy(d_edges, flattenedEdges.data(), edgesSize, cudaMemcpyHostToDevice));
    
    // Initialize distance matrix - use a 2D grid of thread blocks
    dim3 blockSize(16, 16);
    dim3 gridSize((numVertices + blockSize.x - 1) / blockSize.x, 
                 (numVertices + blockSize.y - 1) / blockSize.y);
    
    initializeDistanceMatrix<<<gridSize, blockSize>>>(d_dist, numVertices);
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
    
    // Populate initial edge distances
    int edgeBlockSize = 256;
    int edgeGridSize = (numEdges + edgeBlockSize - 1) / edgeBlockSize;
    
    populateEdges<<<edgeGridSize, edgeBlockSize>>>(d_dist, d_edges, numEdges, numVertices);
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
    
    // Free edge memory as it's no longer needed
    CHECK_CUDA_ERROR(cudaFree(d_edges));
    
    // Get device properties to determine optimal strategy
    cudaDeviceProp deviceProp;
    CHECK_CUDA_ERROR(cudaGetDeviceProperties(&deviceProp, 0));
    
    // Choose algorithm variant based on graph size and GPU capabilities
    if (numVertices <= 2048) {
        // For smaller graphs, use batched k-processing to reduce kernel launches
        int batchSize = 8;  // Process 8 k values at a time
        
        for (int startK = 0; startK < numVertices; startK += batchSize) {
            floydWarshallBatchKernel<<<gridSize, blockSize>>>(d_dist, startK, batchSize, numVertices);
            CHECK_CUDA_ERROR(cudaDeviceSynchronize());
        }
    } 
    else if (numVertices <= 8192) {
        // For medium-sized graphs, use shared memory optimization
        size_t sharedMemSize = 2 * blockSize.x * sizeof(dist_t);
        
        // Main Floyd-Warshall iteration
        for (int k = 0; k < numVertices; k++) {
            floydWarshallKernel<<<gridSize, blockSize, sharedMemSize>>>(d_dist, k, numVertices);
            CHECK_CUDA_ERROR(cudaDeviceSynchronize());
        }
    }
    else {
        // For very large graphs, use tiled approach to handle matrix that doesn't fit in shared memory
        int tileSize = 16;  // 16x16 tiles
        dim3 tileBlockSize(tileSize, tileSize);
        dim3 tileGridSize((numVertices + tileSize - 1) / tileSize, 
                         (numVertices + tileSize - 1) / tileSize);
        size_t tileMem = tileSize * tileSize * sizeof(dist_t);
        
        for (int k = 0; k < numVertices; k++) {
            floydWarshallTiledKernel<<<tileGridSize, tileBlockSize, tileMem>>>(
                d_dist, k, numVertices, tileSize);
            CHECK_CUDA_ERROR(cudaDeviceSynchronize());
        }
    }
    
    // Copy results back to host
    std::vector<dist_t> h_dist(numVertices * numVertices);
    CHECK_CUDA_ERROR(cudaMemcpy(h_dist.data(), d_dist, distSize, cudaMemcpyDeviceToHost));
    
    // Free device memory
    CHECK_CUDA_ERROR(cudaFree(d_dist));
    
    // Convert back to 2D vector format
    std::vector<std::vector<float>> result(numVertices, std::vector<float>(numVertices));
    for (int i = 0; i < numVertices; i++) {
        for (int j = 0; j < numVertices; j++) {
            result[i][j] = h_dist[i * numVertices + j];
        }
    }
    
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
    printf("GPU Floyd-Warshall table generation time %lf seconds\n", duration.count() / 1e6);
    
    return result;
}

// Extended version with adaptive tuning for maximum performance
std::vector<std::vector<float>> FloydWarshall_GPU_Adaptive(
    const std::vector<std::vector<Edge>>& edges,
    int numVertices
) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Flatten edges into a single vector for GPU
    std::vector<Edge> flattenedEdges;
    for (size_t i = 0; i < edges.size(); i++) {
        for (const auto& edge : edges[i]) {
            flattenedEdges.push_back(edge);
        }
    }
    int numEdges = flattenedEdges.size();
    
    // Allocate device memory for distance matrix and edges
    dist_t* d_dist;
    Edge* d_edges;
    size_t distSize = numVertices * numVertices * sizeof(dist_t);
    size_t edgesSize = numEdges * sizeof(Edge);
    
    CHECK_CUDA_ERROR(cudaMalloc(&d_dist, distSize));
    CHECK_CUDA_ERROR(cudaMalloc(&d_edges, edgesSize));
    
    // Copy edges to device
    CHECK_CUDA_ERROR(cudaMemcpy(d_edges, flattenedEdges.data(), edgesSize, cudaMemcpyHostToDevice));
    
    // Initialize distance matrix - use a 2D grid of thread blocks
    dim3 blockSize(16, 16);
    dim3 gridSize((numVertices + blockSize.x - 1) / blockSize.x, 
                 (numVertices + blockSize.y - 1) / blockSize.y);
    
    initializeDistanceMatrix<<<gridSize, blockSize>>>(d_dist, numVertices);
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
    
    // Populate initial edge distances
    int edgeBlockSize = 256;
    int edgeGridSize = (numEdges + edgeBlockSize - 1) / edgeBlockSize;
    
    populateEdges<<<edgeGridSize, edgeBlockSize>>>(d_dist, d_edges, numEdges, numVertices);
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
    
    // Free edge memory as it's no longer needed
    CHECK_CUDA_ERROR(cudaFree(d_edges));
    
    // Get device properties to determine optimal strategy
    cudaDeviceProp deviceProp;
    CHECK_CUDA_ERROR(cudaGetDeviceProperties(&deviceProp, 0));
    
    // Determine memory capacity and compute capability
    size_t availableMem;
    size_t totalMem;
    CHECK_CUDA_ERROR(cudaMemGetInfo(&availableMem, &totalMem));
    
    // Adaptive algorithm selection based on graph size and GPU capabilities
    
    // For very small graphs or low-memory situations, use register-only approach
    if (numVertices <= 1024 || availableMem < 2 * distSize) {
        printf("Using register-optimized batch implementation\n");
        
        // Determine optimal batch size based on compute capability
        int batchSize = (deviceProp.major >= 7) ? 16 : 8;
        
        for (int startK = 0; startK < numVertices; startK += batchSize) {
            int actualBatchSize = std::min(batchSize, numVertices - startK);
            floydWarshallBatchKernel<<<gridSize, blockSize>>>(d_dist, startK, actualBatchSize, numVertices);
            CHECK_CUDA_ERROR(cudaDeviceSynchronize());
        }
    }
    // For medium-sized graphs that fit well in shared memory
    else if (numVertices <= 4096) {
        printf("Using shared memory implementation\n");
        
        // Optimize block size based on shared memory usage
        int optBlockSize = std::min(32, (int)sqrt(deviceProp.sharedMemPerBlock / (2 * sizeof(dist_t))));
        dim3 optBlockDim(optBlockSize, optBlockSize);
        dim3 optGridDim((numVertices + optBlockSize - 1) / optBlockSize, 
                        (numVertices + optBlockSize - 1) / optBlockSize);
        
        size_t sharedMemSize = 2 * optBlockSize * sizeof(dist_t);
        
        // Main Floyd-Warshall iteration
        for (int k = 0; k < numVertices; k++) {
            floydWarshallKernel<<<optGridDim, optBlockDim, sharedMemSize>>>(d_dist, k, numVertices);
            CHECK_CUDA_ERROR(cudaDeviceSynchronize());
        }
    }
    // For large graphs, use tiled approach with optimized tile size
    else {
        printf("Using tiled implementation for large graph\n");
        
        // Determine optimal tile size based on shared memory
        int maxTileSize = (int)sqrt(deviceProp.sharedMemPerBlock / sizeof(dist_t));
        int tileSize = std::min(32, maxTileSize);  // Use at most 32x32 tiles
        
        printf("Using tile size: %d x %d\n", tileSize, tileSize);
        
        dim3 tileBlockSize(tileSize, tileSize);
        dim3 tileGridSize((numVertices + tileSize - 1) / tileSize, 
                         (numVertices + tileSize - 1) / tileSize);
        size_t tileMem = tileSize * tileSize * sizeof(dist_t);
        
        for (int k = 0; k < numVertices; k++) {
            floydWarshallTiledKernel<<<tileGridSize, tileBlockSize, tileMem>>>(
                d_dist, k, numVertices, tileSize);
            CHECK_CUDA_ERROR(cudaDeviceSynchronize());
        }
    }
    
    // Copy results back to host
    std::vector<dist_t> h_dist(numVertices * numVertices);
    CHECK_CUDA_ERROR(cudaMemcpy(h_dist.data(), d_dist, distSize, cudaMemcpyDeviceToHost));
    
    // Free device memory
    CHECK_CUDA_ERROR(cudaFree(d_dist));
    
    // Convert back to 2D vector format
    std::vector<std::vector<float>> result(numVertices, std::vector<float>(numVertices));
    for (int i = 0; i < numVertices; i++) {
        for (int j = 0; j < numVertices; j++) {
            result[i][j] = h_dist[i * numVertices + j];
        }
    }
    
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
    printf("GPU Floyd-Warshall adaptive table generation time %lf seconds\n", duration.count() / 1e6);
    
    return result;
}