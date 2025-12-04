#ifndef OPT_FW_ITL_CPU_PORTABLE_H
#define OPT_FW_ITL_CPU_PORTABLE_H

#include <vector>
#include <limits>
#include <algorithm>
#include <chrono>
#include <string>
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>
#include <omp.h>
#include <filesystem>

namespace fs = std::filesystem;

std::string getExecutablePath_opt_portable()
{
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    std::string executablePath(result, (count > 0) ? count : 0);
    return executablePath.substr(0, executablePath.find_last_of("/"));
}

// ====================== OPTIMIZED FLOYD-WARSHALL ======================

std::vector<std::vector<float>> FloydWarshall_Optimized(
    const std::vector<std::vector<Edge>>& edges, 
    size_t numVertices
) {
    auto start = std::chrono::high_resolution_clock::now();
    
    // 1. Use flat array instead of nested vectors for better cache locality
    float* dist = new float[numVertices * numVertices];
    
    // Initialize to infinity and set diagonal to zero
    #pragma omp parallel for
    for (size_t i = 0; i < numVertices; i++) {
        for (size_t j = 0; j < numVertices; j++) {
            dist[i * numVertices + j] = (i == j) ? 0.0f : std::numeric_limits<float>::max();
        }
    }
    
    // Populate initial edges
    #pragma omp parallel for
    for (size_t i = 0; i < numVertices; i++) {
        for (const auto& edge : edges[i]) {
            size_t targetIdx = edge.getTermVertexID();
            float minDist = edge.getMinDist();
            if (i != targetIdx) { // Skip self-loops
                dist[i * numVertices + targetIdx] = minDist;
            }
        }
    }
    
    // Compute cache-friendly tile size based on approximate L1 cache size
    const size_t L1_CACHE_SIZE = 32768; // 32KB L1 cache (typical)
    const size_t BLOCK_SIZE = std::min(numVertices, 
                          std::max(size_t(16), 
                              size_t(sqrt(L1_CACHE_SIZE / (3 * sizeof(float))))));
    
    // Traditional Floyd-Warshall with tiling for better cache performance
    for (size_t k = 0; k < numVertices; k++) {
        // Process inner loops using tiling for better cache locality
        #pragma omp parallel for schedule(dynamic)
        for (size_t i_block = 0; i_block < numVertices; i_block += BLOCK_SIZE) {
            size_t i_end = std::min(i_block + BLOCK_SIZE, numVertices);
            
            for (size_t j_block = 0; j_block < numVertices; j_block += BLOCK_SIZE) {
                size_t j_end = std::min(j_block + BLOCK_SIZE, numVertices);
                
                for (size_t i = i_block; i < i_end; i++) {
                    #pragma omp simd
                    for (size_t j = j_block; j < j_end; j++) {
                        if (dist[i * numVertices + k] != std::numeric_limits<float>::max() && 
                            dist[k * numVertices + j] != std::numeric_limits<float>::max()) {
                            float newDist = dist[i * numVertices + k] + dist[k * numVertices + j];
                            if (newDist < dist[i * numVertices + j]) {
                                dist[i * numVertices + j] = newDist;
                            }
                        }
                    }
                }
            }
        }
    }
    
    // Convert back to 2D vector format for compatibility
    std::vector<std::vector<float>> result(numVertices, std::vector<float>(numVertices));
    #pragma omp parallel for
    for (size_t i = 0; i < numVertices; i++) {
        std::memcpy(result[i].data(), &dist[i * numVertices], numVertices * sizeof(float));
    }
    
    delete[] dist;
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    printf("Optimized CPU Floyd-Warshall generation time %lf seconds\n", duration.count() / 1e6);

    /* for (size_t i = 0; i < result.size(); ++i) {
        for (size_t j = 0; j < result[i].size(); ++j) {
            if (result[i][j] == std::numeric_limits<float>::max()) {
                std::cout << "INF ";
            } else {
                std::cout << result[i][j] << " ";
            }
        }
        std::cout << std::endl;
    } */
    
    return result;
}

// ====================== OPTIMIZED ITL TABLE GENERATION ======================

// Improved BitSet implementation for efficient set operations
class BitSet {
private:
    unsigned long* bits;
    size_t size;
    size_t numBits;
    
public:
    BitSet() : bits(nullptr), size(0), numBits(0) {
    }
    
    BitSet(size_t n) : numBits(n) {
        size = (n + 63) / 64;
        bits = new unsigned long[size]();
    }
    
    BitSet(const BitSet& other) : numBits(other.numBits), size(other.size) {
        bits = new unsigned long[size];
        memcpy(bits, other.bits, size * sizeof(unsigned long));
    }
    
    BitSet& operator=(const BitSet& other) {
        if (this != &other) {
            if (size != other.size) {
                delete[] bits;
                size = other.size;
                bits = new unsigned long[size];
            }
            numBits = other.numBits;
            memcpy(bits, other.bits, size * sizeof(unsigned long));
        }
        return *this;
    }
    
    ~BitSet() {
        delete[] bits;
    }
    
    void set(size_t idx) {
        if (idx >= numBits) {
            printf("WARNING: BitSet::set() out of bounds access: %zu >= %zu\n", idx, numBits);
            return;
        }
        bits[idx / 64] |= (1UL << (idx % 64));
    }
    
    bool get(size_t idx) const {
        if (idx >= numBits) {
            printf("WARNING: BitSet::get() out of bounds access: %zu >= %zu\n", idx, numBits);
            return false;
        }
        return (bits[idx / 64] & (1UL << (idx % 64))) != 0;
    }
    
    // Clear all bits
    void clear() {
        std::memset(bits, 0, size * sizeof(unsigned long));
    }
    
    // Count number of set bits - optimized version
    size_t count() const {
        size_t count = 0;
        for (size_t i = 0; i < size; i++) {
            unsigned long word = bits[i];
            // Use built-in popcount if available
            #if defined(__GNUC__) || defined(__clang__)
                count += __builtin_popcountl(word);
            #else
                // Fallback manual count
                while (word) {
                    count += (word & 1UL);
                    word >>= 1;
                }
            #endif
        }
        return count;
    }
    
    // Improved forEach method to reliably iterate through set bits
    template<typename Func>
    void forEach(Func f) const {
        for (size_t i = 0; i < size; i++) {
            unsigned long word = bits[i];
            while (word) {
                // Find the position of least significant 1-bit
                #if defined(__GNUC__) || defined(__clang__)
                    int r = __builtin_ctzl(word);          // Count trailing zeros (position of rightmost 1)
                    size_t idx = i * 64 + r;
                    if (idx < numBits) {
                        f(idx);
                    }
                    word &= ~(1UL << r);  // Clear the processed bit
                #else
                    // Fallback approach
                    unsigned long t = word & (~word + 1);  // Isolate the rightmost 1-bit
                    int bit_pos = 0;
                    unsigned long test = 1UL;
                    while ((t & test) == 0) {
                        test <<= 1;
                        bit_pos++;
                    }
                    size_t idx = i * 64 + bit_pos;
                    if (idx < numBits) {
                        f(idx);
                    }
                    word ^= t;  // Clear the rightmost 1-bit
                #endif
            }
        }
    }
    
    // Intersection
    static void intersect(const BitSet& a, const BitSet& b, BitSet& result) {
        size_t minSize = std::min(a.size, b.size);
        for (size_t i = 0; i < minSize; i++) {
            result.bits[i] = a.bits[i] & b.bits[i];
        }
        for (size_t i = minSize; i < result.size; i++) {
            result.bits[i] = 0;
        }
    }
    
    // Union
    static void unite(const BitSet& a, const BitSet& b, BitSet& result) {
        size_t minSize = std::min(a.size, b.size);
        for (size_t i = 0; i < minSize; i++) {
            result.bits[i] = a.bits[i] | b.bits[i];
        }
        
        // Copy remaining bits from the larger set
        if (a.size > minSize) {
            std::memcpy(result.bits + minSize, a.bits + minSize, (a.size - minSize) * sizeof(unsigned long));
        } else if (b.size > minSize) {
            std::memcpy(result.bits + minSize, b.bits + minSize, (b.size - minSize) * sizeof(unsigned long));
        }
    }
};

// Helper function for debugging
bool validateITLResults(const float* ITL, size_t numVertices, const char* phase) {
    bool valid = true;
    int invalidCount = 0;
    
    for (size_t i = 0; i < numVertices && invalidCount < 10; i++) {
        for (size_t j = 0; j < numVertices && invalidCount < 10; j++) {
            float val = ITL[i * numVertices + j];
            // Check for NaN
            if (val != val) {
                printf("WARNING: NaN found at ITL[%zu][%zu] in %s\n", i, j, phase);
                valid = false;
                invalidCount++;
            }
            // Check for negative values (should never happen)
            else if (val < 0 && val != std::numeric_limits<float>::max()) {
                printf("WARNING: Negative value found at ITL[%zu][%zu] = %f in %s\n", i, j, val, phase);
                valid = false;
                invalidCount++;
            }
        }
    }
    
    return valid;
}

std::vector<std::vector<float>> MakeITL_Optimized(
    const std::vector<std::vector<size_t>>& inputVars,
    const std::vector<std::vector<size_t>>& outputVars,
    const std::vector<std::vector<float>>& shortest_paths,
    size_t numVertices,
    std::string tableFilename
) {
    auto start = std::chrono::high_resolution_clock::now();
    
    // Convert shortest_paths to flat array for better cache efficiency
    float* sp_flat = new float[numVertices * numVertices];
    #pragma omp parallel for
    for (size_t i = 0; i < numVertices; i++) {
        for (size_t j = 0; j < numVertices; j++) {
            sp_flat[i * numVertices + j] = shortest_paths[i][j];
        }
    }
    
    // Initialize a flat ITL matrix
    float* ITL = new float[numVertices * numVertices]();
    
    // Create array of BitSets for reachability
    BitSet* reachability = new BitSet[numVertices];
    
    // Initialize each BitSet with the correct size
    for (size_t i = 0; i < numVertices; i++) {
        reachability[i] = BitSet(numVertices);
    }
    
    // Compute reachability matrix
    #pragma omp parallel for
    for (size_t l = 0; l < numVertices; l++) {
        for (size_t m = 0; m < numVertices; m++) {
            if (sp_flat[l * numVertices + m] < std::numeric_limits<float>::max()) {
                reachability[l].set(m);
            }
        }
    }
    
    // Verify reachability matrix is computed correctly
    size_t totalReachable = 0;
    for (size_t i = 0; i < numVertices; i++) {
        size_t count = reachability[i].count();
        totalReachable += count;
        if (count == 0 && i != numVertices-1) {
            printf("WARNING: Vertex %zu cannot reach any other vertices\n", i);
        }
    }
    printf("Average reachable vertices per node: %.2f\n", 
           static_cast<float>(totalReachable) / numVertices);
    
    // Pre-allocate BitSets for set operations
    auto ITL_table_p1_gen_start = std::chrono::high_resolution_clock::now();
    
    // Phase One: Optimized with BitSets and pre-allocation
    BitSet** inputVarSets = new BitSet*[numVertices];
    BitSet** outputVarSets = new BitSet*[numVertices];
    
    // Convert inputVars and outputVars to BitSets for faster operations
    #pragma omp parallel for
    for (size_t i = 0; i < numVertices; i++) {
        inputVarSets[i] = new BitSet(numVertices);
        outputVarSets[i] = new BitSet(numVertices);
        
        for (size_t v : inputVars[i]) {
            inputVarSets[i]->set(v);
        }
        
        for (size_t v : outputVars[i]) {
            outputVarSets[i]->set(v);
        }
    }
    
    // Thread-local storage for temporary BitSets
    #pragma omp parallel
    {
        // Pre-allocate per-thread BitSets for temporary results
        BitSet S_k(numVertices);      // State variables for vertex k (inputs and outputs)
        BitSet U_Sk(numVertices);     // Vertices that can update state vars in S_k
        BitSet temp(numVertices);     // Temporary set for operations
        BitSet X_jk(numVertices);     // Intersection of reachable vertices and U_Sk
        
        #pragma omp for schedule(dynamic)
        for (size_t k = 0; k < numVertices; k++) {
            // Create vertex-k state variable set (union of inputs and outputs)
            BitSet::unite(*inputVarSets[k], *outputVarSets[k], S_k);
            
            // Find vertices that can update any state variables in S_k
            U_Sk.clear();
            
            for (size_t l = 0; l < numVertices; l++) {
                // Check if vertex l's outputs intersect with k's state variables
                BitSet::intersect(*outputVarSets[l], S_k, temp);
                
                if (temp.count() > 0) {
                    U_Sk.set(l);
                }
            }
            
            // Debug U_Sk calculation
            size_t u_sk_count = U_Sk.count();
            if (u_sk_count == 0) {
                printf("WARNING: U_Sk is empty for vertex %zu\n", k);
                
                // Debug S_k
                size_t s_k_count = S_k.count();
                printf("  S_k has %zu elements\n", s_k_count);
                
                // Check if outputs from any vertex intersect with S_k
                size_t intersections = 0;
                for (size_t l = 0; l < numVertices; l++) {
                    BitSet::intersect(*outputVarSets[l], S_k, temp);
                    if (temp.count() > 0) {
                        intersections++;
                    }
                }
                printf("  Found %zu vertices whose outputs intersect with S_k\n", intersections);
            }
            
            // Process each potential earlier-event vertex
            for (size_t j = 0; j < numVertices; j++) {
                // Find vertices reachable from j that can update k's state variables
                BitSet::intersect(reachability[j], U_Sk, X_jk);
                
                // Double-check using the original algorithm's approach
                std::vector<size_t> x_jk_check;
                U_Sk.forEach([&](size_t l) {
                    if (reachability[j].get(l)) {
                        x_jk_check.push_back(l);
                    }
                });
                
                // If the results don't match, use the more conservative one
                if (X_jk.count() != x_jk_check.size()) {
                    printf("WARNING: X_jk calculation mismatch for j=%zu, k=%zu: bit count=%zu, vector size=%zu\n", 
                           j, k, X_jk.count(), x_jk_check.size());
                    
                    // If the original method found vertices but our bit method didn't, use those instead
                    if (X_jk.count() == 0 && !x_jk_check.empty()) {
                        // Find minimum distance manually
                        float minDist = std::numeric_limits<float>::max();
                        for (auto& x_jk : x_jk_check) {
                            float dist = sp_flat[j * numVertices + x_jk];
                            minDist = std::min(minDist, dist);
                        }
                        ITL[j * numVertices + k] = minDist;
                        continue; // Skip the normal processing below
                    }
                }
                
                // If intersection exists, find minimum distance
                if (X_jk.count() > 0) {
                    float minDist = std::numeric_limits<float>::max();
                    size_t iterationCount = 0;
                    
                    X_jk.forEach([&](size_t x_jk) {
                        iterationCount++;
                        if (x_jk >= numVertices) {
                            printf("ERROR: Invalid x_jk index: %zu >= %zu\n", x_jk, numVertices);
                            return;
                        }
                        float dist = sp_flat[j * numVertices + x_jk];
                        minDist = std::min(minDist, dist);
                    });
                    
                    // Verify we processed the expected number of elements
                    if (iterationCount != X_jk.count()) {
                        printf("WARNING: forEach iteration count mismatch: %zu != %zu\n", 
                               iterationCount, X_jk.count());
                    }
                    
                    ITL[j * numVertices + k] = minDist;
                } else {
                    // Vertex j cannot affect vertex k
                    ITL[j * numVertices + k] = std::numeric_limits<float>::max();
                }
            }
        }
    }
    
    auto ITL_table_p1_gen_stop = std::chrono::high_resolution_clock::now();
    auto ITL_table_p1_gen_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        ITL_table_p1_gen_stop - ITL_table_p1_gen_start);
    printf("Optimized ITL table phase 1 generation time %lf seconds\n", ITL_table_p1_gen_duration.count() / 1e6);
    
    // Verify phase 1 results
    validateITLResults(ITL, numVertices, "Phase 1");
    
    // Check how many vertices have ITL[j][k] = infinity after phase 1
    size_t infinityCount = 0;
    for (size_t j = 0; j < numVertices; j++) {
        for (size_t k = 0; k < numVertices; k++) {
            if (ITL[j * numVertices + k] == std::numeric_limits<float>::max()) {
                infinityCount++;
            }
        }
    }
    printf("After phase 1: %zu / %zu ITL entries are infinity (%.2f%%)\n",
           infinityCount, numVertices * numVertices,
           100.0f * infinityCount / (numVertices * numVertices));
    
    // Phase Two: Optimized with BitSets and pre-allocation
    auto ITL_table_p2_gen_start = std::chrono::high_resolution_clock::now();
    
    #pragma omp parallel
    {
        // Pre-allocate per-thread BitSets
        BitSet Z_i(numVertices);   // Vertices that i can affect immediately
        BitSet X_hi(numVertices);  // Intersection of vertices reachable from h and Z_i
        
        #pragma omp for schedule(dynamic)
        for (size_t h = 0; h < numVertices; h++) {
            for (size_t i = 0; i < numVertices; i++) {
                // Find vertices that i can affect immediately (ITL[i][l] == 0)
                Z_i.clear();
                
                for (size_t l = 0; l < numVertices; l++) {
                    if (ITL[i * numVertices + l] == 0) {
                        Z_i.set(l);
                    }
                }
                
                // Skip if Z_i is empty (optimization)
                if (Z_i.count() == 0) {
                    continue;
                }
                
                // Find intersection of reachable vertices from h and Z_i
                BitSet::intersect(reachability[h], Z_i, X_hi);
                
                // Double-check using original approach (similar to phase 1)
                std::vector<size_t> x_hi_check;
                Z_i.forEach([&](size_t l) {
                    if (reachability[h].get(l)) {
                        x_hi_check.push_back(l);
                    }
                });
                
                // Use more conservative approach if results differ
                if (X_hi.count() != x_hi_check.size()) {
                    // If bit method missed intersections that vector method found
                    if (X_hi.count() == 0 && !x_hi_check.empty()) {
                        float minDist = std::numeric_limits<float>::max();
                        for (auto& x_hi : x_hi_check) {
                            float dist = sp_flat[h * numVertices + x_hi];
                            minDist = std::min(minDist, dist);
                        }
                        
                        // Update ITL if phase-two value is smaller
                        if (minDist < ITL[h * numVertices + i]) {
                            ITL[h * numVertices + i] = minDist;
                        }
                        continue;
                    }
                }
                
                // If intersection exists, find minimum distance
                if (X_hi.count() > 0) {
                    float minDist = std::numeric_limits<float>::max();
                    
                    X_hi.forEach([&](size_t x_hi) {
                        float dist = sp_flat[h * numVertices + x_hi];
                        minDist = std::min(minDist, dist);
                    });
                    
                    // Update ITL if phase-two value is smaller
                    if (minDist < ITL[h * numVertices + i]) {
                        ITL[h * numVertices + i] = minDist;
                    }
                }
            }
        }
    }
    
    auto ITL_table_p2_gen_stop = std::chrono::high_resolution_clock::now();
    auto ITL_table_p2_gen_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        ITL_table_p2_gen_stop - ITL_table_p2_gen_start);
    printf("Optimized ITL table phase 2 generation time %lf seconds\n", ITL_table_p2_gen_duration.count() / 1e6);
    
    // Verify phase 2 results
    validateITLResults(ITL, numVertices, "Phase 2");
    
    // Check how many vertices have ITL[j][k] = infinity after phase 2
    infinityCount = 0;
    for (size_t j = 0; j < numVertices; j++) {
        for (size_t k = 0; k < numVertices; k++) {
            if (ITL[j * numVertices + k] == std::numeric_limits<float>::max()) {
                infinityCount++;
            }
        }
    }
    printf("After phase 2: %zu / %zu ITL entries are infinity (%.2f%%)\n",
           infinityCount, numVertices * numVertices,
           100.0f * infinityCount / (numVertices * numVertices));
    
    // Convert to 2D vector format for compatibility
    std::vector<std::vector<float>> result(numVertices, std::vector<float>(numVertices));
    #pragma omp parallel for
    for (size_t i = 0; i < numVertices; i++) {
        for (size_t j = 0; j < numVertices; j++) {
            result[i][j] = ITL[i * numVertices + j];
        }
    }
    
    // Write to CSV
    std::string directory = getExecutablePath_opt_portable() + "/ITL_tables/";
    std::string table_path = directory + tableFilename;

    std::ofstream outFile(table_path);
    for (size_t i = 0; i < numVertices; i++) {
        for (size_t j = 0; j < numVertices; j++) {
            //outFile << (result[i][j] == std::numeric_limits<float>::max() ? "inf" : std::to_string(result[i][j]));
            outFile << result[i][j];
            if (j < numVertices - 1) outFile << ",";
        }
        outFile << "\n";
    }
    outFile.close();
    
    // Clean up
    delete[] sp_flat;
    delete[] ITL;
    delete[] reachability;
    
    for (size_t i = 0; i < numVertices; i++) {
        delete inputVarSets[i];
        delete outputVarSets[i];
    }
    delete[] inputVarSets;
    delete[] outputVarSets;
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    printf("Total optimized ITL generation time %lf seconds\n", duration.count() / 1e6);
    
    return result;
}

#endif // OPT_FW_ITL_CPU_PORTABLE_H