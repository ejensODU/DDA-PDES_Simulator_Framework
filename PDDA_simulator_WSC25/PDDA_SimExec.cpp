#include <algorithm>
#include <iostream>
#include <numeric>
#include <chrono>
#include <cmath>
#include <map>
#include <thread>
#include <omp.h>

#include "PDDA_SimExec.h"
#include "Vertex.h"

// Initialize the static member
std::atomic<size_t> Entity::_entityCount{0};

Entity::Entity()
:_ID(-1), _genTime(-1.0) {}

Entity::Entity(double genTime)
:_ID(_entityCount.fetch_add(1)), _genTime(genTime)
{}

int Entity::getID() const { return _ID; }

void Entity::setExitTime(double exitTime) { _exitTime = exitTime; }


void PDDA_Event::Execute(std::vector<std::shared_ptr<Vertex>>& vertices, PDDA_CalendarQueue& CQ)
{
    vertices[_vertexID]->Run(CQ, _time, _entityID);
    _status = 2;
}


CalendarQueue_Serial::CalendarQueue_Serial(double maxSimTime, double bucketWidth)
:PDDA_CalendarQueue(maxSimTime, 0, 0, 1024), _bucketWidth(bucketWidth), _bucketWidthRecip(1.0/bucketWidth), _numEvents(0) {
    _CQ_Buckets.resize(_numBuckets);
}

void CalendarQueue_Serial::AddInitEvent(int vertexID, double time, int entityID)
{
    AddEvent(vertexID, time, entityID);
}

// Optimized AddEvent method
void CalendarQueue_Serial::AddEvent(int vertexID, double time, int entityID, bool fromFFE) {
    if (time > _maxSimTime) return;
	_numEvents++;
    
    // Precompute bucket index
    size_t unwrapped_index = _CQ_MinIndex + static_cast<size_t>((time - _CQ_MinTime) * _bucketWidthRecip);
    size_t wrapped_index = unwrapped_index & (_numBuckets-1);
    
    // Check if queue needs expansion
    while (unwrapped_index >= _numBuckets && (wrapped_index >= _CQ_MinIndex || unwrapped_index - _CQ_MinIndex >= _numBuckets)) {
        // Double the queue size
        int expansion_index = _numBuckets;
        _numBuckets *= 2;
        _CQ_Buckets.resize(_numBuckets);
        
        // Move buckets to their new positions
        for (size_t i = 0; i < _CQ_MinIndex; i++) {
            _CQ_Buckets[expansion_index + i] = std::move(_CQ_Buckets[i]);
            _CQ_Buckets[i].clear();
        }
        
        // Recalculate wrapped index with new bucket size
        wrapped_index = unwrapped_index & (_numBuckets-1);
    }
    
    auto& events = _CQ_Buckets[wrapped_index];
    
    // Fast path for empty list or end insertion
    if (events.empty() || time >= events.back().getTime()) {
        events.emplace_back(vertexID, time, entityID);
        return;
    }
    
    // Fast path for beginning insertion
    if (time < events.front().getTime()) {
        events.emplace_front(vertexID, time, entityID);
        return;
    }
    
    // Binary search approach for large buckets
    if (events.size() > 10) {
        // Convert list to vector for binary search
        std::vector<double> timeValues;
        timeValues.reserve(events.size());
        for (const auto& event : events) {
            timeValues.push_back(event.getTime());
        }
        
        // Find insertion point using binary search
        auto pos = std::lower_bound(timeValues.begin(), timeValues.end(), time);
        size_t index = std::distance(timeValues.begin(), pos);
        
        // Convert index back to list iterator
        auto it = events.begin();
        std::advance(it, index);
        events.emplace(it, vertexID, time, entityID);
    } 
    else {
        // Linear search for small buckets
        for (auto it = events.begin(); it != events.end(); ++it) {
            if (time < (*it).getTime()) {
                events.emplace(it, vertexID, time, entityID);
                return;
            }
        }
        // Should never reach here due to end check above
        events.emplace_back(vertexID, time, entityID);
    }
}


PDDA_Event CalendarQueue_Serial::GetFirstEvent()
{
    auto& bucket = _CQ_Buckets[_CQ_MinIndex];
    PDDA_Event first_event = std::move(bucket.front());
    bucket.pop_front();
	
	_numEvents--;
    return first_event;
}

bool CalendarQueue_Serial::CheckHasEvents()
{
    bool has_events = false;
    size_t i;
    for (i=0; i<_numBuckets; i++) {
        if (!_CQ_Buckets[(_CQ_MinIndex + i) & (_numBuckets-1)].empty()) {
            has_events = true;
            break;
        }
    }
    _CQ_MinTime += i * _bucketWidth;
    _CQ_MinIndex = (_CQ_MinIndex + i) & (_numBuckets-1);

    return has_events;
}

void CalendarQueue_Serial::PrintCQ() const
{
    size_t i = 1;
    for (int j=0; j<_numBuckets; j++) {
        auto const& bucket = _CQ_Buckets[(_CQ_MinIndex + j) & (_numBuckets-1)];
        for (auto const CQ_event : bucket) {
            int vertex_ID = CQ_event.getVertexID();
            double event_time = CQ_event.getTime();
            int entity_ID = CQ_event.getEntityID();
            printf("\tE (%lu) vertex index: %d, time: %lf, entity index: %d\n", i++, vertex_ID, event_time, entity_ID);
        }
    }
}


CalendarQueue_Serial_Array::CalendarQueue_Serial_Array(double maxSimTime, double bucketWidth)
	: PDDA_CalendarQueue(maxSimTime, 0, 0, 1024), 
	  _bucketWidth(bucketWidth), 
	  _bucketWidthRecip(1.0/bucketWidth),
	  _numEvents(0) {
	// Initialize buckets with arrays of size 10
	_CQ_Buckets.resize(_numBuckets);
	_CQ_BucketOverflows.resize(_numBuckets);
}

void CalendarQueue_Serial_Array::AddInitEvent(int vertexID, double time, int entityID)
{
	AddEvent(vertexID, time, entityID);
}

void CalendarQueue_Serial_Array::AddEvent(int vertexID, double time, int entityID, bool fromFFE)
{
	if (time > _maxSimTime) return;
	_numEvents++;
	
	// Precompute bucket index
	size_t unwrapped_index = _CQ_MinIndex + static_cast<size_t>((time - _CQ_MinTime) * _bucketWidthRecip);
	size_t wrapped_index = unwrapped_index & (_numBuckets-1);
	
	// Check if queue needs expansion
	while (unwrapped_index >= _numBuckets && (wrapped_index >= _CQ_MinIndex || unwrapped_index - _CQ_MinIndex >= _numBuckets)) {
		// Double the queue size
		int expansion_index = _numBuckets;
		_numBuckets *= 2;
		_CQ_Buckets.resize(_numBuckets);
		_CQ_BucketOverflows.resize(_numBuckets);
		
		// Move buckets to their new positions
		for (size_t i = 0; i < _CQ_MinIndex; i++) {
			_CQ_Buckets[expansion_index + i] = std::move(_CQ_Buckets[i]);
			_CQ_BucketOverflows[expansion_index + i] = std::move(_CQ_BucketOverflows[i]);
			
			// Reset the moved buckets
			for (auto& event : _CQ_Buckets[i]) {
				event.resetVertexID();
			}
			_CQ_BucketOverflows[i].clear();
		}
		
		// Recalculate wrapped index with new bucket size
		wrapped_index = unwrapped_index & (_numBuckets-1);
	}
	
	auto& bucket = _CQ_Buckets[wrapped_index];
	auto& overflow = _CQ_BucketOverflows[wrapped_index];
	
	// Create new event
	PDDA_Event newEvent(vertexID, time, entityID);
	
	// Check if there's an empty slot in the array
	bool added = false;
	for (size_t i = 0; i < bucket.size(); i++) {
		if (bucket[i].getVertexID() == -1) {
			bucket[i] = newEvent;
			added = true;
			break;
		}
	}
	
	// If no empty slot, add to overflow list
	if (!added) {
		// If overflow is empty, find a good insertion point based on time
		if (overflow.empty()) {
			// See if we should insert at beginning, middle, or end of array events
			bool inserted = false;
			for (size_t i = 0; i < bucket.size(); i++) {
				if (bucket[i].getVertexID() != -1 && time < bucket[i].getTime()) {
					// Move this event to overflow and put new event in its place
					overflow.push_back(bucket[i]);
					bucket[i] = newEvent;
					inserted = true;
					break;
				}
			}
			
			if (!inserted) {
				overflow.push_back(newEvent);
			}
		} else {
			// Insert into overflow list in time order
			if (time < overflow.front().getTime()) {
				overflow.push_front(newEvent);
			} else if (time >= overflow.back().getTime()) {
				overflow.push_back(newEvent);
			} else {
				// Find insertion point
				for (auto it = overflow.begin(); it != overflow.end(); ++it) {
					if (time < it->getTime()) {
						overflow.insert(it, newEvent);
						break;
					}
				}
			}
		}
	}
}

bool CalendarQueue_Serial_Array::CheckHasEvents()
{
	bool has_events = false;
	size_t i;
	for (i = 0; i < _numBuckets; i++) {
		size_t bucket_index = (_CQ_MinIndex + i) & (_numBuckets-1);
		
		// Check array bucket
		for (const auto& event : _CQ_Buckets[bucket_index]) {
			if (event.getVertexID() != -1) {
				has_events = true;
				break;
			}
		}
		
		// Check overflow bucket if array was empty
		if (!has_events && !_CQ_BucketOverflows[bucket_index].empty()) {
			has_events = true;
			break;
		}
		
		if (has_events) break;
	}
	
	if (has_events) {
		_CQ_MinTime += i * _bucketWidth;
		_CQ_MinIndex = (_CQ_MinIndex + i) & (_numBuckets-1);
	}
	
	return has_events;
}

PDDA_Event CalendarQueue_Serial_Array::GetFirstEvent()
{
	auto& bucket = _CQ_Buckets[_CQ_MinIndex];
	auto& overflow = _CQ_BucketOverflows[_CQ_MinIndex];
	
	// Find the earliest event in the array bucket
	PDDA_Event earliest_event;
	double min_time = std::numeric_limits<double>::max();
	size_t min_index = 0;
	bool found_in_array = false;
	
	for (size_t i = 0; i < bucket.size(); i++) {
		if (bucket[i].getVertexID() != -1 && bucket[i].getTime() < min_time) {
			min_time = bucket[i].getTime();
			min_index = i;
			found_in_array = true;
			earliest_event = bucket[i];
		}
	}
	
	// Check the overflow list if it's not empty
	if (!overflow.empty()) {
		const auto& front_event = overflow.front();
		if (!found_in_array || front_event.getTime() < min_time) {
			// Overflow has the earliest event
			earliest_event = front_event;
			overflow.pop_front();
			
			_numEvents--;
			return earliest_event;
		}
	}
	
	// If we found an event in the array, reset its slot and return it
	if (found_in_array) {
		bucket[min_index].resetVertexID();
		
		_numEvents--;
		return earliest_event;
	}
	
	// This should not happen if CheckHasEvents was called before
	return PDDA_Event();
}

void CalendarQueue_Serial_Array::PrintCQ() const
{
	size_t i = 1;
	for (int j = 0; j < _numBuckets; j++) {
		size_t bucket_index = (_CQ_MinIndex + j) & (_numBuckets-1);
		
		// Print array bucket events
		for (const auto& event : _CQ_Buckets[bucket_index]) {
			if (event.getVertexID() != -1) {
				printf("\tE (%lu) vertex index: %d, time: %lf, entity index: %d\n", 
					   i++, event.getVertexID(), event.getTime(), event.getEntityID());
			}
		}
		
		// Print overflow bucket events
		for (const auto& event : _CQ_BucketOverflows[bucket_index]) {
			printf("\tE (%lu) vertex index: %d, time: %lf, entity index: %d\n", 
				   i++, event.getVertexID(), event.getTime(), event.getEntityID());
		}
	}
}


MultiSet_Serial::MultiSet_Serial(double maxSimTime)
:PDDA_CalendarQueue(maxSimTime, 0, 0, 1024), _numEvents(0) {}

void MultiSet_Serial::AddInitEvent(int vertexID, double time, int entityID)
{
    AddEvent(vertexID, time, entityID);
}

void MultiSet_Serial::AddEvent(int vertexID, double time, int entityID, bool fromFFE)
{
	if (time > _maxSimTime) return;
	_numEvents++;
	
	_MS.insert(std::move(PDDA_Event(vertexID, time, entityID)));
}

PDDA_Event MultiSet_Serial::GetFirstEvent()
{
	_numEvents--;
	
	PDDA_Event first_event = std::move(*_MS.begin());
	_MS.erase(_MS.begin());
	
	return first_event;
}

bool MultiSet_Serial::CheckHasEvents()
{
    return !_MS.empty();
}


CalendarQueue_Parallel::CalendarQueue_Parallel(double maxSimTime, size_t targetBinSize)
:PDDA_CalendarQueue(maxSimTime, 0, 0, 64), _bucketVecSize(targetBinSize), _bucketWidth(1.0), _bucketWidthRecip(1.0/_bucketWidth), _baselineBucketWidth(1.0), _bucketIncrementCount(0), _numEvents(0), _recordItNum(0), _fineAdjustScalar(0.05), _maxNumIncrements(20), _waitPeriod(128), _recordWindow(128), _waitIts(0), _adjustCount(0), _coarseAdjust(true), _farFutureEventsLock(0)
{
    // initialize vector containers
    // Changed: Allocate a single flat array instead of nested vectors
    _CQ_Buckets.resize(_numBuckets * _bucketVecSize);
    
    std::vector<std::atomic<size_t>> new_CQ_BucketIndices(_numBuckets);
    for (size_t i=0; i<_numBuckets; i++) new_CQ_BucketIndices[i].store(0);
    _CQ_BucketIndices = std::move(new_CQ_BucketIndices);
    _CQ_BucketOverflows.resize(_numBuckets);
    _CQ_BucketSizes.resize(_numBuckets);
    _firstBucketNewEvents.resize(_bucketVecSize);
    _recentMaxBucketSizes.resize(_recordWindow);
    std::vector<std::atomic<int>> new_overflowListLocks(_numBuckets);
    for (size_t i=0; i<_numBuckets; i++) new_overflowListLocks[i].store(0);
    _overflowListLocks = std::move(new_overflowListLocks);
}

void CalendarQueue_Parallel::AddInitEvent(int vertexID, double time, int entityID)
{
    _numEvents.fetch_add(1);
    
    // compute wrapped index
    size_t unwrapped_index = _CQ_MinIndex + static_cast<size_t>((time - _CQ_MinTime) * _bucketWidthRecip);
    size_t wrapped_index = unwrapped_index & (_numBuckets-1);  // modulus -> bitwise operator + (2^x - 1) size divisor

    // event does not fit into CQ
    if (unwrapped_index >= _numBuckets && (wrapped_index >= _CQ_MinIndex || unwrapped_index - _CQ_MinIndex >= _numBuckets)) {
    //if (unwrapped_index >= _numBuckets && wrapped_index >= _CQ_MinIndex) {
        _farFutureEvents.emplace_back(vertexID, time, entityID);
    }
    // event fits into CQ
    else {
        size_t next_index = _CQ_BucketIndices[wrapped_index].fetch_add(1);
        // event fits into bucket vector
        if (next_index < _bucketVecSize) {
            // Changed: Calculate the flat array index
            size_t flat_index = wrapped_index * _bucketVecSize + next_index;
            _CQ_Buckets[flat_index] = std::move(PDDA_Event(vertexID, time, entityID));
        }
        // event goes into overflow list
        else {
            _CQ_BucketOverflows[wrapped_index].emplace_back(vertexID, time, entityID);
        }
    }
}

void CalendarQueue_Parallel::AddEvent(int vertexID, double time, int entityID, bool fromFFE)
{
    if (!fromFFE) _numEvents.fetch_add(1);
    
    if (time > _maxSimTime) return;

    // event goes in first bucket, do not currently modify first bucket
    if (time < _CQ_MinTime + _bucketWidth) {
        
        size_t next_index = _CQ_BucketIndices[_CQ_MinIndex].fetch_add(1);
        // event fits into bucket vector
        if (next_index < _bucketVecSize) {
            _firstBucketNewEvents[next_index] = std::move(PDDA_Event(vertexID, time, entityID));
        }
        // event goes into overflow list
        else {
            SpinLockData(_overflowListLocks[_CQ_MinIndex]);
            _CQ_BucketOverflows[_CQ_MinIndex].emplace_back(vertexID, time, entityID);
            UnlockData(_overflowListLocks[_CQ_MinIndex]);
        }
    }
    // event goes in future bucket or far future list, okay to modify
    else {
        // compute wrapped index
        size_t unwrapped_index = _CQ_MinIndex + static_cast<size_t>((time - _CQ_MinTime) * _bucketWidthRecip);
        size_t wrapped_index = unwrapped_index & (_numBuckets-1);  // modulus -> bitwise operator + (2^x - 1) size divisor

        // event does not fit into CQ
        if (unwrapped_index >= _numBuckets && (wrapped_index >= _CQ_MinIndex || unwrapped_index - _CQ_MinIndex >= _numBuckets)) {
        //if (unwrapped_index >= _numBuckets && wrapped_index >= _CQ_MinIndex) {
            SpinLockData(_farFutureEventsLock);
            _farFutureEvents.emplace_back(vertexID, time, entityID);
            UnlockData(_farFutureEventsLock);
        }
        // event fits into CQ
        else {
            size_t next_index = _CQ_BucketIndices[wrapped_index].fetch_add(1);
            // event fits into bucket vector
            if (next_index < _bucketVecSize) {
                // Changed: Calculate the flat array index
                size_t flat_index = wrapped_index * _bucketVecSize + next_index;
                _CQ_Buckets[flat_index] = std::move(PDDA_Event(vertexID, time, entityID));
            }
            // event goes into overflow list
            else {
                SpinLockData(_overflowListLocks[wrapped_index]);
                _CQ_BucketOverflows[wrapped_index].emplace_back(vertexID, time, entityID);
                UnlockData(_overflowListLocks[wrapped_index]);
            }
        }
    }
}

std::vector<PDDA_Event>& CalendarQueue_Parallel::GetFirstBucket(size_t& bucketStartIndex, size_t& partitionPointIndex)
{
    bucketStartIndex = _CQ_MinIndex * _bucketVecSize;
	partitionPointIndex = _partitionPointIndex;
    
    return _CQ_Buckets;
}

size_t CalendarQueue_Parallel::getBucketVecSize()
{
    return _bucketVecSize;
}

size_t CalendarQueue_Parallel::getAdjustCount()
{
    return _adjustCount;
}

double CalendarQueue_Parallel::GetMeanFirstBucketSizes()
{
    size_t sum_sizes = 0;
    for (const auto& size : _firstBucketSizes) sum_sizes += size;
    return static_cast<double>(sum_sizes) / _firstBucketSizes.size();
}

void CalendarQueue_Parallel::UpdateFirstBucket(double& simTime)
{
    // Decrement first-bucket event count, reset executed events, update sim time
    size_t executed_count = 0;
    size_t start_idx = _CQ_MinIndex * _bucketVecSize;
    
    for (size_t j = 0; j < _bucketVecSize; j++) {
        // Changed: Access flat array
        size_t flat_index = start_idx + j;
        if (2 == _CQ_Buckets[flat_index].getStatus()) {
            _numEvents.fetch_sub(1);
            _CQ_BucketSizes[_CQ_MinIndex]--;
            _CQ_Buckets[flat_index].setStatus(-1);
            _CQ_Buckets[flat_index].resetVertexID();
            
            if (_CQ_Buckets[flat_index].getTime() > simTime) {
                simTime = _CQ_Buckets[flat_index].getTime();
            }
            executed_count++;
        }
    }
    //_CQ_BucketIndices.at(_CQ_MinIndex).store(_CQ_BucketSizes.at(_CQ_MinIndex));
    
    // Update first bucket with new events if there are any
    if (_CQ_BucketIndices[_CQ_MinIndex].load() > 0) {
        size_t j = 0;
        // Iterate through new-events vector to look for new events to add (to first-bucket vector or overflow list)
        for (size_t i = 0; i < _bucketVecSize; i++) {
            
            if (-1 != _firstBucketNewEvents[i].getVertexID()) {
                
                _CQ_BucketSizes[_CQ_MinIndex]++;
                
                // Iterate through first-bucket vector to look for empty space for new event
                for (; j < _bucketVecSize; j++) {
                    size_t flat_index = start_idx + j;
                    if (-1 == _CQ_Buckets[flat_index].getStatus()) {
                        _CQ_Buckets[flat_index] = std::move(_firstBucketNewEvents[i]);
                        
                        break;
                    }
                }
                // Add new event to overflow list, if no space remaining in first-bucket vector
                if (j == _bucketVecSize) {
                    _CQ_BucketOverflows[_CQ_MinIndex].push_back(std::move(_firstBucketNewEvents[i]));                    
                }
                // Reset vertex ID in new-event vector
                _firstBucketNewEvents[i].resetVertexID();
				
            } else {
                break;
            }
        }
        // Update first-bucket size and reset first-bucket index
        _CQ_BucketIndices[_CQ_MinIndex].store(0);
    }
    
    // Expand CQ if events in far future list
    if (!_farFutureEvents.empty()) ExpandCQ();
    
    // Update first-bucket size and reset first-bucket index
    //_CQ_BucketIndices.at(_CQ_MinIndex).store(0);

    // Find overflowing buckets, find bucket with max size, and update sizes
    std::list<size_t> overflow_bucket_indices;
    size_t max_bucket_size = 0;
    for (size_t i = 0; i < _numBuckets; i++) {
        if (i != _CQ_MinIndex) {
            _CQ_BucketSizes[i] = _CQ_BucketIndices[i].load();
        }
        if (_CQ_BucketSizes[i] > _bucketVecSize) {
            overflow_bucket_indices.push_back(i);
        }
        if (_CQ_BucketSizes[i] > max_bucket_size) {
            max_bucket_size = _CQ_BucketSizes[i];
        }
    }
    
    _recentMaxBucketSizes[_recordItNum & (_recordWindow-1)] = max_bucket_size;
    size_t max_recent_max_bucket_sizes = _bucketVecSize;
    if (++_recordItNum >= _recordWindow) max_recent_max_bucket_sizes = *std::max_element(_recentMaxBucketSizes.begin(), _recentMaxBucketSizes.end());

    // Adjust bucket width if necessary
    if (max_bucket_size > _bucketVecSize ||
        (_waitIts++ > _waitPeriod && max_recent_max_bucket_sizes < 0.9*_bucketVecSize)) {// && _numEvents.load() > 10*max_bucket_size)) {
		
        AdjustBucketWidth(max_bucket_size, overflow_bucket_indices);
        _adjustCount++;
        
        if (std::find(_bucketWidthHistory.begin(), _bucketWidthHistory.end(), _bucketWidth) != _bucketWidthHistory.end()) {
            _waitIts = 0;
            //_waitPeriod *= 2;
        }
        _recordItNum = 0;
        _bucketWidthHistory.push_back(_bucketWidth);
    }
}

bool CalendarQueue_Parallel::CheckHasEvents(bool init)
{
    // find first non-empty bucket
    bool has_events = false;
    size_t i;
    for (i=0; i<_numBuckets; i++) {
        for (size_t j=0; j<_bucketVecSize; j++) {
            // Changed: Calculate flat array index
            size_t flat_index = ((_CQ_MinIndex + i) & (_numBuckets-1)) * _bucketVecSize + j;
            if (0 == _CQ_Buckets[flat_index].getStatus()) {
                has_events = true;
                break;
            }
        }
        if (has_events) break;
    }
    // update min time, index
    _CQ_MinTime += i * _bucketWidth;
    _CQ_MinIndex = (_CQ_MinIndex + i) & (_numBuckets-1);
	
	//if (i>0) printf("updated CQ min index\n");
    
    if (init) {
        _CQ_BucketSizes[_CQ_MinIndex] = _CQ_BucketIndices[_CQ_MinIndex];
        _CQ_BucketIndices[_CQ_MinIndex].store(0);
    }
    
    if (i > 0) {
        _firstBucketSizes.push_back(_CQ_BucketSizes[_CQ_MinIndex]);
		_CQ_BucketIndices[_CQ_MinIndex].store(0);
    }
	
	// Define the range for the first bucket
    auto bucketBegin = _CQ_Buckets.begin() + _CQ_MinIndex * _bucketVecSize;
    auto bucketEnd = bucketBegin + _bucketVecSize;
    
    // Partition the bucket to move events with _vertexID == -1 to the end
    auto partitionPoint = std::partition(bucketBegin, bucketEnd, 
                                        [](const PDDA_Event& event) { 
                                            return event.getVertexID() > -1; 
                                        });
    
    // Sort the valid events (with _vertexID > -1) by time in ascending order
    std::sort(bucketBegin, partitionPoint, 
             [](const PDDA_Event& a, const PDDA_Event& b) { 
                 return a.getTime() < b.getTime(); 
             });
    
    // Calculate and update the partition point index (relative position in _CQ_Buckets)
    _partitionPointIndex = partitionPoint - bucketBegin;

    return has_events;
}

void CalendarQueue_Parallel::ExpandCQ()
{
    size_t num_first_bucket_events = _CQ_BucketSizes[_CQ_MinIndex];

    // get max-timestamp event
    auto max_iter = std::max_element(_farFutureEvents.begin(), _farFutureEvents.end(),
                                     [](const PDDA_Event& a, const PDDA_Event& b) {
                                         return a.getTime() < b.getTime();
                                     });

    // compute wrapped index
    double max_TS = max_iter->getTime();
    size_t max_unwrapped_index = _CQ_MinIndex + static_cast<size_t>((max_TS - _CQ_MinTime) * _bucketWidthRecip);
    size_t max_wrapped_index = max_unwrapped_index & (_numBuckets-1);  // modulus -> bitwise operator + (2^x - 1) size divisor

    // expand CQ until farthest-future event fits
    while (max_unwrapped_index >= _numBuckets && (max_wrapped_index >= _CQ_MinIndex || max_unwrapped_index - _CQ_MinIndex >= _numBuckets)) {
    //while (max_unwrapped_index >= _numBuckets && max_wrapped_index >= _CQ_MinIndex) {
        size_t expansion_index = _numBuckets;
        size_t old_numBuckets = _numBuckets;
        _numBuckets *= 2;
        
        // Changed: Create a new flat array with more buckets
        std::vector<PDDA_Event> new_CQ_Buckets(_numBuckets * _bucketVecSize);
        
        std::vector<std::atomic<size_t>> new_CQ_BucketIndices(_numBuckets);
        for (size_t i=0; i<_numBuckets; i++) new_CQ_BucketIndices[i].store(0);
        
        _CQ_BucketOverflows.resize(_numBuckets);
        _CQ_BucketSizes.resize(_numBuckets);
        std::vector<std::atomic<int>> new_overflowListLocks(_numBuckets);
        for (size_t i=0; i<_numBuckets; i++) new_overflowListLocks[i].store(0);
        _overflowListLocks = std::move(new_overflowListLocks);

        // Copy bucket indices for buckets from Min to the end of old array
        for (size_t i=_CQ_MinIndex; i<old_numBuckets; i++) {
            new_CQ_BucketIndices[i].store(_CQ_BucketIndices[i].load());
            
            // Copy events from old buckets to new buckets
            for (size_t j=0; j<_bucketVecSize; j++) {
                size_t old_flat_index = i * _bucketVecSize + j;
                size_t new_flat_index = i * _bucketVecSize + j;
                new_CQ_Buckets[new_flat_index] = std::move(_CQ_Buckets[old_flat_index]);
            }
        }

        // Move buckets from beginning to Min to the expanded area
        for (size_t i=0; i<_CQ_MinIndex; i++) {
            size_t new_bucket_idx = expansion_index + i;
            new_CQ_BucketIndices[new_bucket_idx].store(_CQ_BucketIndices[i].load());
            
            // Copy events from old buckets to new expanded buckets
            for (size_t j=0; j<_bucketVecSize; j++) {
                size_t old_flat_index = i * _bucketVecSize + j;
                size_t new_flat_index = new_bucket_idx * _bucketVecSize + j;
                new_CQ_Buckets[new_flat_index] = std::move(_CQ_Buckets[old_flat_index]);
            }
            
            // Move overflow lists
            _CQ_BucketOverflows[expansion_index + i] = std::move(_CQ_BucketOverflows[i]);
        }
        
        _CQ_Buckets = std::move(new_CQ_Buckets);
        _CQ_BucketIndices = std::move(new_CQ_BucketIndices);
        
        max_wrapped_index = max_unwrapped_index & (_numBuckets-1);  // recalculate with new bucket count
    }

    // add far-future events
    for (const auto& event : _farFutureEvents) AddEvent(event.getVertexID(), event.getTime(), event.getEntityID(), true);

    // clear far future events list
    _farFutureEvents.clear();
}

void CalendarQueue_Parallel::AdjustBucketWidth(size_t maxBucketSize, std::list<size_t>& overflowBucketIndices)
{   
    // Calculate size ratio compared to target size
    double size_ratio = static_cast<double>(maxBucketSize) / _bucketVecSize;

    // Skip if within acceptable bounds
    if (size_ratio >= 0.9 && size_ratio <= 1.0) return;

    double adjustment_factor;
    double new_bucket_width;

    // too many events per bucket
    if (size_ratio > 1.0) {
        // coarse adjust, decrease bucket width
        if (_coarseAdjust) {
            // Initial adjustment using bit_ceil for power of 2 division
            adjustment_factor = 1.0 / std::bit_ceil(static_cast<unsigned int>(std::ceil(size_ratio)));
            
            bool events_fit;
            do {
                events_fit = true;
                std::map<size_t, size_t> new_bucket_counts;
                
                // Only need to verify buckets that currently have overflow events
                for (size_t bucket_idx=0; bucket_idx<_numBuckets; bucket_idx++) {
                    // Calculate how overflow events would be distributed with new bucket width
                    // First count events from main bucket
                    for (size_t i = 0; i < _bucketVecSize; i++) {
                        size_t event_idx = bucket_idx * _bucketVecSize + i;
                        if (_CQ_Buckets[event_idx].getVertexID() != -1 && _CQ_Buckets[event_idx].getStatus() != 2) {
                            double delta_time = _CQ_Buckets[event_idx].getTime() - _CQ_MinTime;
                            size_t new_bucket = static_cast<size_t>(delta_time / (_baselineBucketWidth * adjustment_factor));
                            new_bucket_counts[new_bucket]++;
                        }
                    }
                    // Then verify overflow events will fit
                    for (const auto& event : _CQ_BucketOverflows[bucket_idx]) {
                        double delta_time = event.getTime() - _CQ_MinTime;
                        size_t new_bucket = static_cast<size_t>(delta_time / (_baselineBucketWidth * adjustment_factor));
                        new_bucket_counts[new_bucket]++;
                        
                        if (new_bucket_counts[new_bucket] > _bucketVecSize) {
                            events_fit = false;
                            break;
                        }
                    }
                    
                    if (!events_fit) break;
                }
                
                if (!events_fit) {
                    adjustment_factor *= 0.5;  // Try a smaller adjustment if events don't fit
                }
                else {
                    // Calculate new bucket width
                    new_bucket_width = _baselineBucketWidth * adjustment_factor;
                    _baselineBucketWidth = new_bucket_width;
                }
                
            } while (!events_fit);
        }
        // fine adjust, decrease bucket width
        else {
            bool events_fit;
            
            do {
                
                if (0 == _bucketIncrementCount) {
                    _baselineBucketWidth *= 0.5;
                    _bucketIncrementCount = _maxNumIncrements-1;
                }
                else _bucketIncrementCount--;
                
                adjustment_factor = (1.0 + _fineAdjustScalar*_bucketIncrementCount);
                
                events_fit = true;
                std::map<size_t, size_t> new_bucket_counts;
                
                for (size_t bucket_idx=0; bucket_idx<_numBuckets; bucket_idx++) {
                    // Calculate how overflow events would be distributed with new bucket width
                    // First count events from main bucket
                    for (size_t i = 0; i < _bucketVecSize; i++) {
                        size_t event_idx = bucket_idx * _bucketVecSize + i;
                        if (_CQ_Buckets[event_idx].getVertexID() != -1 && _CQ_Buckets[event_idx].getStatus() != 2) {
                            double delta_time = _CQ_Buckets[event_idx].getTime() - _CQ_MinTime;
                            size_t new_bucket = static_cast<size_t>(delta_time / (_baselineBucketWidth * adjustment_factor));
                            new_bucket_counts[new_bucket]++;
                            
                            if (new_bucket_counts[new_bucket] > _bucketVecSize) {
                                events_fit = false;
                                break;
                            }
                        }
                    }
                    // Then verify overflow events will fit
                    for (const auto& event : _CQ_BucketOverflows[bucket_idx]) {
                        double delta_time = event.getTime() - _CQ_MinTime;
                        size_t new_bucket = static_cast<size_t>(delta_time / (_baselineBucketWidth * adjustment_factor));
                        new_bucket_counts[new_bucket]++;
                        
                        if (new_bucket_counts[new_bucket] > _bucketVecSize) {
                            events_fit = false;
                            break;
                        }
                    }
                    if (!events_fit) {
                        break;
                    }
                }
                
            } while (!events_fit);
                
            // Calculate new bucket width
            new_bucket_width = _baselineBucketWidth * adjustment_factor;
        }
    }
    // too few events per bucket
    else {
        // coarse adjust, increase bucket width
        if (_coarseAdjust && size_ratio < 0.5) {
            adjustment_factor = 2.0;
            new_bucket_width = _baselineBucketWidth * adjustment_factor;
            _baselineBucketWidth = new_bucket_width;
        }
        // fine adjust, increase bucket width
        else {
            _coarseAdjust = false;
            
            if (_maxNumIncrements == ++_bucketIncrementCount) {
                    _baselineBucketWidth *= 2;
                    _bucketIncrementCount = 0;
            }
            
            adjustment_factor = (1.0 + _fineAdjustScalar*_bucketIncrementCount);
            
            bool events_fit = true;
            std::map<size_t, size_t> new_bucket_counts;
            
            for (size_t bucket_idx=0; bucket_idx<_numBuckets; bucket_idx++) {
                // Calculate how overflow events would be distributed with new bucket width
                // First count events from main bucket
                for (size_t i = 0; i < _bucketVecSize; i++) {
                    size_t event_idx = bucket_idx * _bucketVecSize + i;
                    if (_CQ_Buckets[event_idx].getVertexID() != -1 && _CQ_Buckets[event_idx].getStatus() != 2) {
                        double delta_time = _CQ_Buckets[event_idx].getTime() - _CQ_MinTime;
                        size_t new_bucket = static_cast<size_t>(delta_time / (_baselineBucketWidth * adjustment_factor));
                        new_bucket_counts[new_bucket]++;
                        
                        if (new_bucket_counts[new_bucket] > _bucketVecSize) {
                            events_fit = false;
                            break;
                        }
                    }
                }
                // Then verify overflow events will fit
                for (const auto& event : _CQ_BucketOverflows[bucket_idx]) {
                    double delta_time = event.getTime() - _CQ_MinTime;
                    size_t new_bucket = static_cast<size_t>(delta_time / (_baselineBucketWidth * adjustment_factor));
                    new_bucket_counts[new_bucket]++;
                    
                    if (new_bucket_counts[new_bucket] > _bucketVecSize) {
                        events_fit = false;
                        break;
                    }
                }
                
                if (!events_fit) break;
            }
            if (events_fit) {
                new_bucket_width = _baselineBucketWidth * adjustment_factor;
            }
            else {
                if (0 == _bucketIncrementCount) {
                    _baselineBucketWidth *= 0.5;
                    _bucketIncrementCount = _maxNumIncrements-1;
                }
                
                return;
            }
        }
        
    }
    double new_bucket_width_recip = 1.0 / new_bucket_width;

    // Create temporary storage for redistribution
    std::vector<PDDA_Event> new_buckets(_numBuckets * _bucketVecSize);
    std::vector<std::atomic<size_t>> new_bucket_indices(_numBuckets);
    for (auto& index : new_bucket_indices) {
        index.store(0);
    }

    // Redistribute all events into main buckets only (no overflows needed)
    for (size_t i = 0; i < _numBuckets; i++) {
        size_t current_bucket = (_CQ_MinIndex + i) & (_numBuckets-1);
        
        // Redistribute main bucket events
        for (size_t j = 0; j < _bucketVecSize; j++) {
            size_t event_idx = current_bucket * _bucketVecSize + j;
            if (_CQ_Buckets[event_idx].getVertexID() != -1 && _CQ_Buckets[event_idx].getStatus() != 2) {
                double delta_time = _CQ_Buckets[event_idx].getTime() - _CQ_MinTime;

                size_t new_unwrapped_index = _CQ_MinIndex + static_cast<size_t>(delta_time * new_bucket_width_recip);
                size_t new_wrapped_index = new_unwrapped_index & (_numBuckets-1);

                // event does not fit into CQ
                if (new_unwrapped_index >= _numBuckets && (new_wrapped_index >= _CQ_MinIndex || new_unwrapped_index - _CQ_MinIndex >= _numBuckets)) {
                    _farFutureEvents.push_back(std::move(_CQ_Buckets[event_idx]));
                }
                else {
                    size_t next_index = new_bucket_indices[new_wrapped_index].fetch_add(1);
                    size_t new_event_idx = new_wrapped_index * _bucketVecSize + next_index;
                    new_buckets[new_event_idx] = std::move(_CQ_Buckets[event_idx]);
                }
            }
        }
        
        // Redistribute overflow events
        for (auto& event : _CQ_BucketOverflows[current_bucket]) {
            double delta_time = event.getTime() - _CQ_MinTime;
            
            size_t new_unwrapped_index = _CQ_MinIndex + static_cast<size_t>(delta_time * new_bucket_width_recip);
            size_t new_wrapped_index = new_unwrapped_index & (_numBuckets-1);

            // event does not fit into CQ
            if (new_unwrapped_index >= _numBuckets && (new_wrapped_index >= _CQ_MinIndex || new_unwrapped_index - _CQ_MinIndex >= _numBuckets)) {
                _farFutureEvents.push_back(std::move(event));
            }
            else {
                size_t next_index = new_bucket_indices[new_wrapped_index].fetch_add(1);
                size_t new_event_idx = new_wrapped_index * _bucketVecSize + next_index;
                new_buckets[new_event_idx] = std::move(event);
            }
        }
    }

    // Update calendar queue state
    _bucketWidth = new_bucket_width;
    _bucketWidthRecip = new_bucket_width_recip;
    _CQ_Buckets = std::move(new_buckets);
    _CQ_BucketIndices = std::move(new_bucket_indices);
    _CQ_BucketOverflows.clear();
    _CQ_BucketOverflows.resize(_numBuckets);  // Reset all overflow lists to empty

    // Update bucket sizes (now just the number of events in main buckets)
    for (size_t i = 0; i < _numBuckets; i++) {
        _CQ_BucketSizes[i] = _CQ_BucketIndices[i].load();
    }
    // First-bucket events are already moved over, reset index
    _CQ_BucketIndices[_CQ_MinIndex].store(0);

    // Expand CQ if events in far future list
    if (!_farFutureEvents.empty()) ExpandCQ();
}

void CalendarQueue_Parallel::SpinLockData(std::atomic<int>& shared_lock)
{
    int expected = 0;
    while (!shared_lock.compare_exchange_weak(expected, -1)) {
        expected = 0;
        std::this_thread::yield();
    }
}

void CalendarQueue_Parallel::UnlockData(std::atomic<int>& shared_lock)
{
    shared_lock.store(0);
}

void CalendarQueue_Parallel::PrintCQ() const
{
    size_t i = 1;
    for (int j = 0; j < _numBuckets; j++) {
        int idx = (_CQ_MinIndex + j) & (_numBuckets-1);
		for (int k = 0; k < _bucketVecSize; k++) {
            int vertex_ID = _CQ_Buckets[idx+k].getVertexID();
            double event_time = _CQ_Buckets[idx+k].getTime();
            int entity_ID = _CQ_Buckets[idx+k].getEntityID();
            printf("\tE (%lu) vertex index: %d, time: %lf, entity index: %d\n", i++, vertex_ID, event_time, entity_ID);
        }
    }
}


PDDA_SimExec::PDDA_SimExec(size_t numThreads, std::vector<std::shared_ptr<Vertex>>& vertices, std::vector<std::vector<float>> ITL, double maxSimTime, double bucketWidth, size_t targetBinSize)
:_numThreads(numThreads), _vertices(vertices), _ITL(ITL)
{
	if (0 == _numThreads) _MS_Ser = std::make_unique<MultiSet_Serial>(maxSimTime);
    else if (1 == _numThreads) _CQ_Ser = std::make_unique<CalendarQueue_Serial>(maxSimTime, bucketWidth);
	else if (999 == _numThreads) _CQ_Ser_Array = std::make_unique<CalendarQueue_Serial_Array>(maxSimTime, bucketWidth);
    else _CQ_Par = std::make_unique<CalendarQueue_Parallel>(maxSimTime, targetBinSize);
}


void PDDA_SimExec::ScheduleInitEvent(const PDDA_Event& initEvent)
{
    if (0 == _numThreads) _MS_Ser->AddInitEvent(initEvent.getVertexID(), initEvent.getTime(), initEvent.getEntityID());
	else if (1 == _numThreads) _CQ_Ser->AddInitEvent(initEvent.getVertexID(), initEvent.getTime(), initEvent.getEntityID());
	else if (999 == _numThreads) _CQ_Ser_Array->AddInitEvent(initEvent.getVertexID(), initEvent.getTime(), initEvent.getEntityID());
    else _CQ_Par->AddInitEvent(initEvent.getVertexID(), initEvent.getTime(), initEvent.getEntityID());
}

void PDDA_SimExec::RunSim()
{
	if (0 == _numThreads) RunSerialSim_MS();
    else if (1 == _numThreads) RunSerialSim();
	else if (999 == _numThreads) RunSerialSim_CQ_Array();
    else RunParallelSim();
}

void PDDA_SimExec::RunSerialSim_MS()
{
    auto start_exec_serial_MS = std::chrono::high_resolution_clock::now();

    double sim_time = 0;
    size_t exec_count = 0;
    bool run = _MS_Ser->CheckHasEvents();

    while (run) {
        PDDA_Event first_event = _MS_Ser->GetFirstEvent();
        first_event.Execute(_vertices, *_MS_Ser);  // Dereference the unique_ptr
        sim_time = first_event.getTime();
        exec_count++;
        run = _MS_Ser->CheckHasEvents();
    }
    auto stop_exec_serial_MS = std::chrono::high_resolution_clock::now();
    auto duration_exec_serial_MS = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_exec_serial_MS - start_exec_serial_MS);

    std::cout << "\nMS serial sim time: " << sim_time << ", num events executed: " << exec_count << std::endl;
    std::cout << "MS SERIAL runtime: " << duration_exec_serial_MS.count() * 1e-9 << " seconds" << std::endl;
}

void PDDA_SimExec::RunSerialSim()
{
    auto start_exec_serial = std::chrono::high_resolution_clock::now();

    double sim_time = 0;
    size_t exec_count = 0;
    bool run = _CQ_Ser->CheckHasEvents();

    while (run) {
        PDDA_Event first_event = _CQ_Ser->GetFirstEvent();
        first_event.Execute(_vertices, *_CQ_Ser);  // Dereference the unique_ptr
        sim_time = first_event.getTime();
        exec_count++;
        run = _CQ_Ser->CheckHasEvents();
    }
    auto stop_exec_serial = std::chrono::high_resolution_clock::now();
    auto duration_exec_serial = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_exec_serial - start_exec_serial);

    std::cout << "\nserial sim time: " << sim_time << ", num events executed: " << exec_count << std::endl;
    std::cout << "SERIAL runtime: " << duration_exec_serial.count() * 1e-9 << " seconds" << std::endl;
}

void PDDA_SimExec::RunSerialSim_CQ_Array()
{
    auto start_exec_serial_CQ_array = std::chrono::high_resolution_clock::now();

    double sim_time = 0;
    size_t exec_count = 0;
    bool run = _CQ_Ser_Array->CheckHasEvents();

    while (run) {
        PDDA_Event first_event = _CQ_Ser_Array->GetFirstEvent();
        first_event.Execute(_vertices, *_CQ_Ser_Array);  // Dereference the unique_ptr
        sim_time = first_event.getTime();
        exec_count++;
        run = _CQ_Ser_Array->CheckHasEvents();
    }
    auto stop_exec_serial_CQ_array = std::chrono::high_resolution_clock::now();
    auto duration_exec_serial_CQ_array = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_exec_serial_CQ_array - start_exec_serial_CQ_array);

    std::cout << "\nCQ array serial sim time: " << sim_time << ", num events executed: " << exec_count << std::endl;
    std::cout << "CQ array SERIAL runtime: " << duration_exec_serial_CQ_array.count() * 1e-9 << " seconds" << std::endl;
}

void PDDA_SimExec::RunParallelSim()
{
    auto start_exec_parallel = std::chrono::high_resolution_clock::now();

    double sim_time = 0;
    std::atomic<size_t> exec_count = 0;
    //size_t bucket_vec_size = _CQ_Par->getBucketVecSize();
	bool run = _CQ_Par->CheckHasEvents(true);
    _CQ_Par->UpdateFirstBucket(sim_time);
    size_t it_num = 0;
	
	std::atomic<size_t> it_RE_count = 0;
	std::atomic<size_t> it_event_count = 0;
	size_t total_RE_count = 0;
	size_t total_event_count = 0;

    omp_set_num_threads(_numThreads);

	#pragma omp parallel
	{
		int num_threads = omp_get_num_threads();
		int tid = omp_get_thread_num();
		size_t bucket_start_index;
		size_t bucket_vec_size;

		while (run) {
			
			std::vector<PDDA_Event>& CQ_buckets = _CQ_Par->GetFirstBucket(bucket_start_index, bucket_vec_size);

			#pragma omp for schedule(dynamic)
			for (size_t j=0; j<bucket_vec_size; j++) {

				if (0 == CQ_buckets[bucket_start_index + j].getStatus()) {

					bool event_ready = true;
					int spec_RE_vertex_ID = CQ_buckets[bucket_start_index + j].getVertexID();
					double spec_RE_timestamp = CQ_buckets[bucket_start_index + j].getTime();
					
					it_event_count.fetch_add(1);

					for (size_t i=0; i<j; i++) {
						
						int spec_dep_event_vertex_ID = CQ_buckets[bucket_start_index + i].getVertexID();
						double spec_dep_event_timestamp = CQ_buckets[bucket_start_index + i].getTime();

						// tranpose ITL table?
						float indep_limit = _ITL[spec_RE_vertex_ID][spec_dep_event_vertex_ID];
						//float indep_limit = _ITL[spec_dep_event_vertex_ID][spec_RE_vertex_ID];
						if (spec_RE_timestamp - spec_dep_event_timestamp >= static_cast<double>(indep_limit)) {
							event_ready = false;
							
							//printf("\tconflicting event %lu: vertex ID %d, time %lf\n", i, spec_dep_event_vertex_ID, spec_dep_event_timestamp);
							
							break;
						}
					}
					if (event_ready) {
						it_RE_count.fetch_add(1);
						CQ_buckets[bucket_start_index + j].Execute(_vertices, *_CQ_Par);
						exec_count.fetch_add(1);
						
						//printf("tid %d executed event %lu\n", tid, j);
					}
					else {
						//printf("tid %d DID NOT execute event %lu\n", tid, j);
					}
				}
				else {
					//printf("tid %d skipped event %lu\n", tid, j);
				}

			}

			#pragma omp single
			{
				_CQ_Par->UpdateFirstBucket(sim_time);
				run = _CQ_Par->CheckHasEvents();
				
				total_RE_count += it_RE_count.load();
				total_event_count += it_event_count.load();

				//printf("It. %lu: exec %lu/%lu, total %lf%%, \tsim time %lf, exec count %lu\n", 
				//	it_num, it_RE_count.load(), it_event_count.load(), 
				//	static_cast<double>(total_RE_count) / static_cast<double>(total_event_count) * 100.0,
				//	sim_time, exec_count.load());

				it_num++;
				it_RE_count = 0;
				it_event_count = 0;
			}
		}
	}

    auto stop_exec_parallel = std::chrono::high_resolution_clock::now();
    auto duration_exec_parallel = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_exec_parallel - start_exec_parallel);

    std::cout << "\nparallel sim time: " << sim_time << ", num events executed: " << exec_count << ", RE efficiency: " << static_cast<double>(total_RE_count) / static_cast<double>(total_event_count) * 100.0 << ", num its: " << it_num << ", adjust count: " << _CQ_Par->getAdjustCount() << std::endl;
    std::cout << "PARALLEL runtime: " << duration_exec_parallel.count() * 1e-9 << " seconds" << std::endl;
	
	printf("Mean first-bucket size: %lf\n", _CQ_Par->GetMeanFirstBucketSizes());
}
