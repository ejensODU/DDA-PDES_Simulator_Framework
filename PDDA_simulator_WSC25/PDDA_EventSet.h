#pragma once

#include <vector>
#include <list>
#include <set>
#include <memory>
#include <atomic>

class Vertex;

struct EventRecord {
    size_t _sequenceNum;
    double _timestamp;
    std::string _eventType;
};

class Entity {
public:
    Entity(double genTime);
    void setExitTime(double exitTime);
    virtual ~Entity() = default;  // Virtual destructor for proper cleanup of derived classes.
    virtual void PrintData() const = 0;
protected:
    const size_t _ID;
    const double _genTime;
    double _exitTime;
private:
    static std::atomic<size_t> _entityCount;
};

class PDDA_Event {
public:
    PDDA_Event(std::shared_ptr<Vertex> vertex, double time, std::shared_ptr<Entity> entity);
    PDDA_Event(const PDDA_Event& other);
    void Execute();
    std::list<PDDA_Event*>& getNewEvents()  { return _newEvents;}
    double getTime() const  { return _time; }
    std::shared_ptr<Vertex> getVertex()  { return _vertex; }
	int getVertexIndex() const;
	void setStatus(int status) { _status.store(status); }
	int getStatus() const  { return _status.load(); }
private:
    std::shared_ptr<Vertex> _vertex;
    const double _time;
    std::shared_ptr<Entity> _entity;
    std::atomic<int> _status;
    std::list<PDDA_Event*> _newEvents;
};

struct EventPtr_Compare final
{
    bool operator() (const std::shared_ptr<PDDA_Event> left, const std::shared_ptr<PDDA_Event> right) const
    {
        if (left->getTime() < right->getTime()) return true;
        if (right->getTime() < left->getTime()) return false;

        return left->getVertexIndex() < right->getVertexIndex();
    }
};


class PDDA_EventSet {
public:
    PDDA_EventSet(std::vector<std::vector<float>> ITL, double maxSimTime);
    void GetReadyEvents(std::list<std::shared_ptr<PDDA_Event>>& readyEvents);
    bool UpdateEventSet(double& simTime);
    void AddEvent(PDDA_Event* newEvent);
    bool GetEmpty() const  { return _E.empty(); }
    int GetSize() const  { return _E.size(); }
    void ExecuteSerial(double& simTime, std::atomic<int>& numEventsExecuted, std::string execOrderFilename);
    void ExecuteSerial_PDDA(double& simTime, std::atomic<int>& numEventsExecuted, int distSeed, int numSerialPDDA_Execs, std::string IO_ExecOrderFilename);
    void GetReadyEventsPDDA_Serial(std::list<std::shared_ptr<PDDA_Event>>& readyEvents, unsigned short& numReadyEvents, double& meanReadyEventIndex, double& stdReadyEventIndex, std::string& readyEventNames);
    void CountReadyEventsIO_Serial(unsigned short& numReadyEvents, double& meanReadyEventIndex, double& stdReadyEventIndex, std::string& readyEventNames);
    void PrintE();
    double GetReadyEventsMeanSize();
    double GetE_SizesMeanSize();
    double GetE_RangesMeanRange();
    void WriteSerialReadyEventsToCSV();
    //void WriteParallelReadyEventsToCSV();
    void PrintParallelReadyEventsStats();
private:
    std::multiset<std::shared_ptr<PDDA_Event>, EventPtr_Compare> _E;
    std::multiset<std::shared_ptr<PDDA_Event>, EventPtr_Compare> _copyInitE;
    std::vector<std::vector<float>> _ITL;
    std::list<PDDA_Event*> _newEvents;
    std::shared_ptr<PDDA_Event> _eLater;
    std::shared_ptr<PDDA_Event> _eEarlier;
    int _eeVertInd, _leVertInd;
    bool _leIndep;
    double _eeLeLimit;
    const int _omega;
    const double _maxSimTime;
    std::list<unsigned short> _readyEventsSizes;
    std::list<unsigned short> _E_Sizes;
    std::list<unsigned short> _E_Ranges;
    // serial testing
    std::list<unsigned short> _numReadyEventsSerial;
    std::list<double> _readyEventIndexMeans;
    std::list<double> _readyEventIndexStds;
    std::list<std::string> _readyEventNames;
    std::vector<double> _maxTS_ByEventType;
    // parallel testing
    std::vector<unsigned short> _numReadyByEventType;
    std::list<unsigned short> _numReadyArriveEvents;
    std::list<unsigned short> _numReadyLandEvents;
    std::list<unsigned short> _numReadyDepartEvents;
    std::vector<unsigned short> _numNewReadyByEventType;
    std::list<unsigned short> _numNewReadyArriveEvents;
    std::list<unsigned short> _numNewReadyLandEvents;
    std::list<unsigned short> _numNewReadyDepartEvents;
    std::list<long long int>  _readyEventWCTs;

};
