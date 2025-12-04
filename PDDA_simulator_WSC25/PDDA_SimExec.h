#pragma once

#include <atomic>
#include <memory>
#include <vector>
#include <array>
#include <list>
#include <set>
#include <utility>

class Vertex;
class PDDA_CalendarQueue;

class Entity {
public:
    Entity();
    Entity(double genTime);
    int getID() const;
    void setExitTime(double exitTime);
    virtual ~Entity() = default;  // Virtual destructor for proper cleanup of derived classes.
    virtual void PrintData() const = 0;
protected:
    double _genTime;
    double _exitTime;
    int _ID;
private:
    static std::atomic<size_t> _entityCount;
};
 
// Streamlined event class
class PDDA_Event {
public:
    // Default constructor
    PDDA_Event() : _vertexID(-1), _time(-1.0), _entityID(-1), _status(-1) {}
    
    // Main constructor
    PDDA_Event(int vertexID, double time, int entityID)
        : _vertexID(vertexID), _time(time), _entityID(entityID), _status(0) {}
    
    // Copy constructor
    PDDA_Event(const PDDA_Event& other)
        : _vertexID(other._vertexID), 
          _time(other._time), 
          _entityID(other._entityID), 
          _status(other._status) {}
    
    // Copy assignment - use copy-and-swap idiom for exception safety
    PDDA_Event& operator=(PDDA_Event other) {
        swap(*this, other);
        return *this;
    }
    
    // Move constructor
    PDDA_Event(PDDA_Event&& other) noexcept
        : _vertexID(std::exchange(other._vertexID, -1)),
          _time(std::exchange(other._time, -1.0)),
          _entityID(std::exchange(other._entityID, -1)),
          _status(other._status) {}
		  
	// Comparison operators
    bool operator<(const PDDA_Event& other) const {
        if (_time < other._time) return true;
        if (other._time < _time) return false;
        return _vertexID < other._vertexID;
    }
	
	bool operator==(const PDDA_Event& other) const {
        return _time == other._time && _vertexID == other._vertexID && _entityID == other._entityID;
    }
    
    // Move assignment handled by copy-and-swap idiom
    
    // Swap function for copy-and-swap idiom
    friend void swap(PDDA_Event& first, PDDA_Event& second) noexcept {
        using std::swap;
        swap(first._vertexID, second._vertexID);
        swap(first._time, second._time);
        swap(first._entityID, second._entityID);
        int temp = second._status;
        second._status = first._status;
        first._status = temp;
    }
    
    void Execute(std::vector<std::shared_ptr<Vertex>>& vertices, PDDA_CalendarQueue& CQ);
    
    // Getters and setters
    int getVertexID() const { return _vertexID; }
    void resetVertexID() { _vertexID = -1; }
    double getTime() const { return _time; }
    int getEntityID() const { return _entityID; }
    void setStatus(size_t status) { _status = status; }
    int getStatus() const { return _status; }
    
private:
    double _time;
    int _vertexID;
    int _entityID;
    int _status;
};
 
class PDDA_CalendarQueue {
public:
    PDDA_CalendarQueue(double maxSimTime, double CQ_MinTime, size_t CQ_MinIndex, size_t numBuckets) : _maxSimTime(maxSimTime), _CQ_MinTime(CQ_MinTime), _CQ_MinIndex(CQ_MinIndex), _numBuckets(numBuckets) {};
    virtual void AddInitEvent(int vertexID, double time, int entityID) = 0;
    virtual void AddEvent(int vertexID, double time, int entityID, bool fromFFE=false) = 0;
protected:
    double _maxSimTime;
    double _CQ_MinTime;
    size_t _CQ_MinIndex;
    size_t _numBuckets;
};

class CalendarQueue_Serial : public PDDA_CalendarQueue {
public:
    CalendarQueue_Serial(double maxSimTime, double bucketWidth);
    virtual void AddInitEvent(int vertexID, double time, int entityID);
    virtual void AddEvent(int vertexID, double time, int entityID, bool fromFFE=false);
    bool CheckHasEvents();
    PDDA_Event GetFirstEvent();
    void PrintCQ() const;
private:
    std::vector<std::list<PDDA_Event>> _CQ_Buckets;
    const double _bucketWidth;
    const double _bucketWidthRecip;
    size_t _numEvents;
};

class CalendarQueue_Serial_Array : public PDDA_CalendarQueue {
public:
    CalendarQueue_Serial_Array(double maxSimTime, double bucketWidth);
    virtual void AddInitEvent(int vertexID, double time, int entityID);
    virtual void AddEvent(int vertexID, double time, int entityID, bool fromFFE=false);
    bool CheckHasEvents();
    PDDA_Event GetFirstEvent();
    void PrintCQ() const;
private:
    std::vector<std::array<PDDA_Event, 2>> _CQ_Buckets;
	std::vector<std::list<PDDA_Event>> _CQ_BucketOverflows;
    const double _bucketWidth;
    const double _bucketWidthRecip;
    size_t _numEvents;
};

class MultiSet_Serial : public PDDA_CalendarQueue {
public:
    MultiSet_Serial(double maxSimTime);
    virtual void AddInitEvent(int vertexID, double time, int entityID);
    virtual void AddEvent(int vertexID, double time, int entityID, bool fromFFE=false);
    bool CheckHasEvents();
    PDDA_Event GetFirstEvent();
    void PrintMS() const;
private:
	std::multiset<PDDA_Event> _MS;
    size_t _numEvents;
};

class CalendarQueue_Parallel : public PDDA_CalendarQueue {
public:
    CalendarQueue_Parallel(double maxSimTime, size_t targetBinSize);
    virtual void AddInitEvent(int vertexID, double time, int entityID);
    virtual void AddEvent(int vertexID, double time, int entityID, bool fromFFE=false);
	std::vector<PDDA_Event>& GetFirstBucket(size_t& bucketStartIndex, size_t& partitionPointIndex);
    //std::vector<PDDA_Event>& GetFirstBucket(size_t& bucketStartIndex);
    void UpdateFirstBucket(double& simTime);
    bool CheckHasEvents(bool init=false);
    double GetMeanFirstBucketSizes();
    void PrintCQ() const;
    size_t getBucketVecSize();
    size_t getAdjustCount();
private:
    void ExpandCQ();
    void AdjustBucketWidth(size_t maxBucketSize, std::list<size_t>& overflowBucketIndices);
    bool TryLockData(std::atomic<int>& shared_lock);
    void SpinLockData(std::atomic<int>& shared_lock);
    void UnlockData(std::atomic<int>& shared_lock);
    // Changed from 2D to 1D vector
    std::vector<PDDA_Event> _CQ_Buckets;
    std::vector<std::atomic<size_t>> _CQ_BucketIndices;
    std::vector<std::list<PDDA_Event>> _CQ_BucketOverflows;
    std::vector<size_t> _CQ_BucketSizes;
    std::vector<PDDA_Event> _firstBucketNewEvents;
    std::vector<size_t> _recentMaxBucketSizes;
    std::vector<double> _bucketWidthHistory;
    std::vector<std::atomic<int>> _overflowListLocks;
    std::list<PDDA_Event> _farFutureEvents;
    std::list<size_t> _firstBucketSizes;
    std::atomic<size_t> _numEvents;
    const double _fineAdjustScalar;
    double _bucketWidth;
    double _bucketWidthRecip;
    double _baselineBucketWidth;
    const size_t _bucketVecSize;
    const size_t _waitPeriod;
    const size_t _recordWindow;
    const size_t _maxNumIncrements;
	size_t _partitionPointIndex;
    size_t _bucketIncrementCount;
    size_t _recordItNum;
    size_t _waitIts;
    size_t _adjustCount;
    std::atomic<int> _farFutureEventsLock;
    bool _coarseAdjust;
};

class PDDA_SimExec {
public:
    PDDA_SimExec(size_t numThreads, std::vector<std::shared_ptr<Vertex>>& vertices, std::vector<std::vector<float>> ITL, double maxSimTime, double bucketWidth, size_t targetBinSize);
    void ScheduleInitEvent(const PDDA_Event& initEvent);
    void RunSim();
private:
	void RunSerialSim_MS();
    void RunSerialSim();
	void RunSerialSim_CQ_Array();
    void RunParallelSim();
    const std::vector<std::vector<float>> _ITL;
    std::vector<std::shared_ptr<Vertex>>& _vertices;
	std::unique_ptr<MultiSet_Serial> _MS_Ser;
    std::unique_ptr<CalendarQueue_Serial> _CQ_Ser;
	std::unique_ptr<CalendarQueue_Serial_Array> _CQ_Ser_Array;
    std::unique_ptr<CalendarQueue_Parallel> _CQ_Par;
    const size_t _numThreads;
};