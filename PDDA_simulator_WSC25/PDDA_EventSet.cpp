#include "PDDA_EventSet.h"
#include "PDDA_SimModel.h"

#include <iostream>
#include <fstream>
#include <numeric>
#include <chrono>
#include <cmath>
#include <random>
#include <algorithm>
#include <memory>


// Initialize the static member
std::atomic<size_t> Entity::_entityCount{0};

Entity::Entity(double genTime)
:_ID(_entityCount.fetch_add(1)), _genTime(genTime)
{}

void Entity::setExitTime(double exitTime) { _exitTime = exitTime; }


PDDA_Event::PDDA_Event(std::shared_ptr<Vertex> vertex, double time, std::shared_ptr<Entity> entity)
:_vertex(vertex), _time(time), _entity(entity)
{}

// Implementation of the copy constructor
PDDA_Event::PDDA_Event(const PDDA_Event& other)
    : _vertex(other._vertex),
      _time(other._time),
      _status(other._status.load()),  // Copy the value of the atomic variable
      _newEvents(other._newEvents)
{
    // Initialize the atomic variable (or handle as needed)
    _status.store(other._status.load());
}

void PDDA_Event::Execute()
{
    _vertex->Run(_newEvents, _time, _entity);
}

int PDDA_Event::getVertexIndex() const  { return _vertex->getVertexIndex(); }


PDDA_EventSet::PDDA_EventSet(std::vector<std::vector<float>> ITL, double maxSimTime)
:_ITL(ITL), _maxSimTime(maxSimTime), _omega(32)
{
    _maxTS_ByEventType = std::vector<double>(3,0);
}

void PDDA_EventSet::AddEvent(PDDA_Event* newEvent)
{
    std::shared_ptr<PDDA_Event> sharedPtr(newEvent);
    _E.insert(std::move(sharedPtr));
}


void PDDA_EventSet::GetReadyEvents(std::list<std::shared_ptr<PDDA_Event>>& readyEvents)
{
    int i = 0;
    // iterate through event set, from start to end
    for (std::multiset<std::shared_ptr<PDDA_Event>, EventPtr_Compare>::iterator later_it = _E.begin(); later_it != _E.end(); later_it++) {
        // stop at omega if event set is too large
        if (i++ == _omega) break;
        // get later event object
        _eLater = (*later_it);
        // status 0 is idle event

        // non-0 means ready or completed (atomic)
        if (0 != _eLater->getStatus()) {

            //// testing parallel capacity, 3 vertices
            //if (1 == _eLater->getStatus()) _numReadyByEventType.at(_eLater->getVertexIndex() % 3) += 1;

            continue;
        }

        // later event is independent?
        _leIndep = true;
        // get vertex index of later event
        _leVertInd = _eLater->getVertexIndex();
        // iterate through event set, from beginning to before later event
        for(std::multiset<std::shared_ptr<PDDA_Event>, EventPtr_Compare>::iterator earlier_it = _E.begin(); earlier_it != later_it; earlier_it++) {
            // get earlier event object
            _eEarlier = (*earlier_it);
            // get vertex index of earlier event
            _eeVertInd = _eEarlier->getVertexIndex();
            // get ITL-table limit of event pair
            _eeLeLimit = static_cast<double>(_ITL[_eeVertInd][_leVertInd]);
            // if event pair is not independent, later event is not independent in event set
            if (_eLater->getTime() - _eEarlier->getTime() >= _eeLeLimit) {
                _leIndep = false;  break;
            }
        }
        // if later event is independent
        if (_leIndep) {
            // status of later event ready (atomic)
            _eLater->setStatus(1);
            // put a pointer to later event in ready event set
            readyEvents.push_back(_eLater);

            //// testing parallel capacity, 3 vertices
            //_numNewReadyByEventType.at(_leVertInd % 3) += 1;
            //_numReadyByEventType.at(_leVertInd % 3) += 1;
        }
    }

    //// testing parallel capacity, 3 vertices
    //_numReadyArriveEvents.push_back(_numReadyByEventType.at(0));
    //_numReadyLandEvents.push_back(_numReadyByEventType.at(1));
    //_numReadyDepartEvents.push_back(_numReadyByEventType.at(2));

    //_numNewReadyArriveEvents.push_back(_numNewReadyByEventType.at(0));
    //_numNewReadyLandEvents.push_back(_numNewReadyByEventType.at(1));
    //_numNewReadyDepartEvents.push_back(_numNewReadyByEventType.at(2));

    //_readyEventWCTs.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
    //_E_Sizes.push_back(_E.size());
    //_E_Ranges.push_back((*_E.rbegin())->getTime() - (*_E.begin())->getTime());

    //if (readyEvents.size() > 0) _readyEventsSizes.push_back(readyEvents.size());
    
    return;  
}


bool PDDA_EventSet::UpdateEventSet(double& simTime)
{
    int i = 0;
    // iterate through the event set
    for (auto it = _E.begin(); it != _E.end(); ) {
        // event executed, ready for removal (atomic)
        if (2 == (*it)->getStatus()) {
            // get new events from executed event
            _newEvents = (*it)->getNewEvents();
            // schedule new events in event set
            for (auto& eventPtr : _newEvents) {
                std::shared_ptr<PDDA_Event> sharedPtr(eventPtr);
                _E.insert(std::move(sharedPtr));
            }
            // empty new events container
            _newEvents.clear();
            // update simulation-clock time
            if ((*it)->getTime() > simTime) simTime = (*it)->getTime();
            // remove executed event, do not inc it
            it = _E.erase(it);
        }
        // if not removing event, increment iterator
        else ++it;
        i++;
    }
    // if (!_E.empty()) {
    //     _E_Sizes.push_back(_E.size());
    //     _E_Ranges.push_back((*_E.rbegin())->getTime() - (*_E.begin())->getTime());
    // }

    // simulation will terminate if event set is empty
    return !_E.empty() && simTime <= _maxSimTime;
}



void PDDA_EventSet::ExecuteSerial(double& simTime, std::atomic<int>& numEventsExecuted, std::string execOrderFilename)
{
    unsigned short numReadyEvents;
    double meanReadyEventIndex;
    double stdReadyEventIndex;

    // std::ofstream exec_order_file(execOrderFilename);
    // exec_order_file << "event_sequence_num, timestamp, event_type" << std::endl;

    while (!_E.empty() && (*_E.begin())->getTime() <= _maxSimTime) {

        _E_Sizes.push_back(_E.size());
        
        std::shared_ptr<PDDA_Event> first_event = *_E.begin();
        //std::cout << first_event->getVertex()->getVertexName() << " " << numEventsExecuted.load() << std::endl;
        first_event->Execute();


        //vert_name = first_event->getVertex()->getVertName();

        //for (auto& eventPtr : first_event->getNewEvents()) _E.insert(std::move(eventPtr));

        for (auto& eventPtr : first_event->getNewEvents()) {
            std::shared_ptr<PDDA_Event> sharedPtr(eventPtr);
            _E.insert(std::move(sharedPtr));
          }
        
        simTime = first_event->getTime();
        size_t event_index = numEventsExecuted.fetch_add(1);

        // exec_order_file << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)
        // << event_index << ", " << simTime << ", "
        // << first_event->getVertex()->getVertexName() << std::endl;
        
        _E.erase(_E.begin());

        if (!_E.empty()) {
             _E_Sizes.push_back(_E.size());
             _E_Ranges.push_back((*_E.rbegin())->getTime() - (*_E.begin())->getTime());

             // std::string readyEventNames;
             // CountReadyEventsSerial(numReadyEvents, meanReadyEventIndex, stdReadyEventIndex, readyEventNames);
             // _readyEventsSizes.push_back(numReadyEvents);
             // _numReadyEventsSerial.push_back(numReadyEvents);
             // _readyEventIndexMeans.push_back(meanReadyEventIndex);
             // _readyEventIndexStds.push_back(stdReadyEventIndex);
             // _readyEventNames.push_back(readyEventNames);

             //std::cout << simTime << "\t" << vert_name << "\t" << numReadyEvents << std::endl;

             //_readyEventWCTs.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
        }
    }
    return;
}


void PDDA_EventSet::ExecuteSerial_PDDA(double& simTime, std::atomic<int>& numEventsExecuted, int distSeed, int numSerialPDDA_Execs, std::string IO_ExecOrderFilename)
{
    std::list<std::shared_ptr<PDDA_Event>> ready_events;
    unsigned short num_ready_events;
    double mean_ready_event_index;
    double std_ready_event_index;

    // size_t num_event_matches = 0;
    // std::vector<size_t> index_diffs;
    // std::vector<EventRecord> IO_events;
    // std::ifstream exec_order_file(IO_ExecOrderFilename);
    //
    // std::string line;
    // std::getline(exec_order_file, line); // header
    // while (std::getline(exec_order_file, line)) {
    //     std::stringstream ss(line);
    //     std::string token;
    //
    //     // Parse sequence number
    //     std::getline(ss, token, ',');
    //     size_t seq = std::stoi(token);
    //
    //     // Parse timestamp
    //     std::getline(ss, token, ',');
    //     double timestamp = std::stod(token);
    //
    //     // Parse event type
    //     std::getline(ss, token, ',');
    //     // Remove leading/trailing whitespace
    //     token.erase(0, token.find_first_not_of(" \t"));
    //     token.erase(token.find_last_not_of(" \t") + 1);
    //
    //     //printf("seq num: %lu, timestamp: %lf, type: %s\n", seq, timestamp, token.c_str());
    //
    //     IO_events.emplace_back(seq, timestamp, token);
    // }

    std::ofstream RE_sets("RE_sets.txt");

    std::mt19937 rng(distSeed);  // Initialize the random number generator with distSeed

    while (!_E.empty() && (*_E.begin())->getTime() <= _maxSimTime) {

        ready_events.clear();
        std::string ready_event_names;
        GetReadyEventsPDDA_Serial(ready_events, num_ready_events, mean_ready_event_index, std_ready_event_index, ready_event_names);
        _readyEventsSizes.push_back(num_ready_events);
        _numReadyEventsSerial.push_back(num_ready_events);
        _readyEventIndexMeans.push_back(mean_ready_event_index);
        _readyEventIndexStds.push_back(std_ready_event_index);
        _readyEventNames.push_back(ready_event_names);
        _E_Sizes.push_back(_E.size());

        if (numSerialPDDA_Execs > 0) {
            int num_events_exec = 0;
            for (std::shared_ptr<PDDA_Event>& event : ready_events) {
                event->Execute();
                event->setStatus(2);
                size_t event_count = numEventsExecuted.fetch_add(1);

                if (++num_events_exec == std::pow(2, numSerialPDDA_Execs)) break;
            }
        } else {
            double percentage = -numSerialPDDA_Execs * 10.0;
            int num_random_events = ceil((num_ready_events * percentage) / 100.0);

            std::vector<std::shared_ptr<PDDA_Event>> ready_events_vector(ready_events.begin(), ready_events.end());
            std::shuffle(ready_events_vector.begin(), ready_events_vector.end(), rng);

            for (int i = 0; i < num_random_events; ++i) {
                ready_events_vector[i]->Execute();
                ready_events_vector[i]->setStatus(2);

                double timestamp = ready_events_vector[i]->getTime();
                std::string vertex_name = ready_events_vector[i]->getVertex()->getVertexName();

                RE_sets << "(" << vertex_name << "," << timestamp << ")" << ",";

                // size_t event_index = numEventsExecuted.fetch_add(1);
                // double timestamp = ready_events_vector[i]->getTime();
                // std::string vertex_name = ready_events_vector[i]->getVertex()->getVertexName();
                //
                // if (IO_events.at(event_index)._timestamp == timestamp && IO_events.at(event_index)._eventType == vertex_name) num_event_matches++;
                //
                // for (const auto& event_record : IO_events) {
                //
                //     if (event_record._timestamp == timestamp && event_record._eventType == vertex_name) index_diffs.push_back(abs(event_index-event_record._sequenceNum));
                // }
            }
            RE_sets << std::endl;
        }
        UpdateEventSet(simTime);
    }

    // double sum_diffs = std::accumulate(index_diffs.begin(), index_diffs.end(), 0.0);
    // double mean_diffs = sum_diffs / index_diffs.size();
    // double square_sum_diffs = std::inner_product(
    //     index_diffs.begin(), index_diffs.end(),
    //                                              index_diffs.begin(), 0.0,
    //                                              std::plus<>(),
    //                                              [mean_diffs](size_t a, size_t b) { return (a - mean_diffs) * (b - mean_diffs); }
    // );
    // double std_diffs = std::sqrt(square_sum_diffs / index_diffs.size());
    //
    // std::ofstream match_count_file(IO_ExecOrderFilename);
    // match_count_file << "num_event_matches, mean_diffs, std_diffs" << std::endl;
    // match_count_file << num_event_matches << ", " << mean_diffs << ", " << std_diffs << std::endl;

    return;
}


void PDDA_EventSet::GetReadyEventsPDDA_Serial(std::list<std::shared_ptr<PDDA_Event>>& readyEvents, unsigned short& numReadyEvents, double& meanReadyEventIndex, double& stdReadyEventIndex, std::string& readyEventNames)
{
    //std::fill(_numReadyByEventType.begin(), _numReadyByEventType.end(), 0);
    //std::fill(_maxTS_ByEventType.begin(), _maxTS_ByEventType.end(), 0.0);

    std::ofstream E_sets("E_sets.txt", std::ios::app);

    numReadyEvents = 0;
    std::list<int> ready_event_indices;
    int i = 0;
    // iterate through event set, from start to end
    for (std::multiset<std::shared_ptr<PDDA_Event>, EventPtr_Compare>::iterator later_it = _E.begin(); later_it != _E.end(); later_it++) {
        // stop at omega if event set is too large
        //if (i++ == _omega) break;
        // get later event object
        _eLater = (*later_it);
        // status 0 is idle event
        // non-0 means ready or completed (atomic)
        //if (0 != _eLater->getStatus()) continue;
        // later event is independent?
        _leIndep = true;
        // get vertex index of later event
        _leVertInd = _eLater->getVertexIndex();

        double timestamp = _eLater->getTime();
        std::string vertex_name = _eLater->getVertex()->getVertexName();

        E_sets << "(" << vertex_name << "," << timestamp << ")" << ",";

        //if (_eLater->getTime() > _maxTS_ByEventType.at(_leVertInd % 3)) _maxTS_ByEventType.at(_leVertInd % 3) = _eLater->getTime();

        // iterate through event set, from beginning to before later event
        for(std::multiset<std::shared_ptr<PDDA_Event>, EventPtr_Compare>::iterator earlier_it = _E.begin(); earlier_it != later_it; earlier_it++) {
            // get earlier event object
            _eEarlier = (*earlier_it);
            // get vertex index of earlier event
            _eeVertInd = _eEarlier->getVertexIndex();
            // get ITL-table limit of event pair
            _eeLeLimit = static_cast<double>(_ITL[_eeVertInd][_leVertInd]);
            // if event pair is not independent, later event is not independent in event set
            if (_eLater->getTime() - _eEarlier->getTime() >= _eeLeLimit) {
                _leIndep = false;  break;
            }
        }
        // if later event is independent
        if (_leIndep) {
            // status of later event ready (atomic)
            _eLater->setStatus(1);
            // put a pointer to later event in ready event set
            readyEvents.push_back(_eLater);
            //std::cout << "found ready event: " << i << ", curr num ready events: " << numReadyEvents << std::endl;
            numReadyEvents++;
            ready_event_indices.push_back(i);

            //_numReadyByEventType.at(_leVertInd % 3) += 1;
        }
        i++;
    }
    E_sets << std::endl;

    meanReadyEventIndex = std::accumulate(ready_event_indices.begin(), ready_event_indices.end(), 0.0) / ready_event_indices.size();

    double variance = std::accumulate(ready_event_indices.begin(), ready_event_indices.end(), 0.0, [meanReadyEventIndex](double sum, unsigned short value) { return sum + std::pow(value - meanReadyEventIndex, 2);}) / ready_event_indices.size();
    stdReadyEventIndex = std::sqrt(variance);

    readyEventNames.append(" ");

    //std::cout << numReadyEvents << std::endl;

    //std::cout << "\tA:" << _numReadyByEventType.at(0) << "\tL:" << _numReadyByEventType.at(1) << "\tD:" << _numReadyByEventType.at(2) << std::endl;
    //std::cout << "\tA:" << _maxTS_ByEventType.at(0) << "\tL:" << _maxTS_ByEventType.at(1) << "\tD:" << _maxTS_ByEventType.at(2) << std::endl;

    return;
}


void PDDA_EventSet::CountReadyEventsIO_Serial(unsigned short& numReadyEvents, double& meanReadyEventIndex, double& stdReadyEventIndex, std::string& readyEventNames)
{
    //std::fill(_numReadyByEventType.begin(), _numReadyByEventType.end(), 0);
    //std::fill(_maxTS_ByEventType.begin(), _maxTS_ByEventType.end(), 0.0);

    numReadyEvents = 0;
    std::list<int> ready_event_indices;
    int i = 0;
    // iterate through event set, from start to end
    for (std::multiset<std::shared_ptr<PDDA_Event>, EventPtr_Compare>::iterator later_it = _E.begin(); later_it != _E.end(); later_it++) {
        // stop at omega if event set is too large
        //if (i++ == _omega) break;
        // get later event object
        _eLater = (*later_it);
        // status 0 is idle event
        // non-0 means ready or completed (atomic)
        //if (0 != _eLater->getStatus()) continue;
        // later event is independent?
        _leIndep = true;
        // get vertex index of later event
        _leVertInd = _eLater->getVertexIndex();

        // bool rp_vertex = false;
        //
        // std::string vertex_name = _eLater->getVertex()->getVertexName();
        // size_t position = vertex_name.find("ready plane");
        // if (position != std::string::npos) {
        //     rp_vertex = true;
        //     std::cout << "\trp vertex index: " << _leVertInd << std::endl;
        // }

        if (_eLater->getTime() > _maxTS_ByEventType.at(_leVertInd % 3)) _maxTS_ByEventType.at(_leVertInd % 3) = _eLater->getTime();

        // iterate through event set, from beginning to before later event
        for(std::multiset<std::shared_ptr<PDDA_Event>, EventPtr_Compare>::iterator earlier_it = _E.begin(); earlier_it != later_it; earlier_it++) {
            // get earlier event object
            _eEarlier = (*earlier_it);
            // get vertex index of earlier event
            _eeVertInd = _eEarlier->getVertexIndex();
            // get ITL-table limit of event pair
            _eeLeLimit = static_cast<double>(_ITL[_eeVertInd][_leVertInd]);
            // if event pair is not independent, later event is not independent in event set
            if (_eLater->getTime() - _eEarlier->getTime() >= _eeLeLimit) {

                // if (rp_vertex) {
                //     std::cout << "\tready plane event is dependent :-(" << std::endl;
                //     std::cout << "\t\trdy pln later vertex: " << _eLater->getVertex()->getVertexName() << ", ts: " << _eLater->getTime() << std::endl;
                //     std::cout << "\t\tdepnd earlier vertex: " << _eEarlier->getVertex()->getVertexName() << ", ts: " << _eEarlier->getTime() << std::endl << std::endl;
                // }

                _leIndep = false;  break;
            }
        }
        // if later event is independent
        if (_leIndep) {

            //if (rp_vertex) std::cout << "\tready plane event is INdependent!" << std::endl;

            // status of later event ready (atomic)
            //_eLater->setStatus(1);
            // put a pointer to later event in ready event set
            //readyEvents.push_back(_eLater);
            //std::cout << "found ready event: " << i << ", curr num ready events: " << numReadyEvents << std::endl;
            numReadyEvents++;
            ready_event_indices.push_back(i);
            //readyEventNames.append(_eLater->getVertex()->getVertexName()).append(";");

            //_numReadyByEventType.at(_leVertInd % 3) += 1;
        }
        i++;
    }
    meanReadyEventIndex = std::accumulate(ready_event_indices.begin(), ready_event_indices.end(), 0.0) / ready_event_indices.size();

    double variance = std::accumulate(ready_event_indices.begin(), ready_event_indices.end(), 0.0, [meanReadyEventIndex](double sum, unsigned short value) { return sum + std::pow(value - meanReadyEventIndex, 2);}) / ready_event_indices.size();
    stdReadyEventIndex = std::sqrt(variance);

    readyEventNames.append(" ");

    //std::cout << "\tA:" << _numReadyByEventType.at(0) << "\tL:" << _numReadyByEventType.at(1) << "\tD:" << _numReadyByEventType.at(2) << std::endl;
    //std::cout << "\tA:" << _maxTS_ByEventType.at(0) << "\tL:" << _maxTS_ByEventType.at(1) << "\tD:" << _maxTS_ByEventType.at(2) << std::endl;

    return;
}


void PDDA_EventSet::PrintE()
{
    int i = 1;
    for (auto E_event : _E) {
        int vertex_index = E_event->getVertexIndex();
        std::string vertex_name = E_event->getVertex()->getVertexName();
        double event_time = E_event->getTime();

        printf("\tE (%d) vertex name: %s, vertex index: %d, time: %lf\n", i++,
        vertex_name.c_str(), vertex_index, event_time);
    }
}

double PDDA_EventSet::GetReadyEventsMeanSize()
{
    return std::accumulate(_readyEventsSizes.begin(), _readyEventsSizes.end(), 0) / double(_readyEventsSizes.size());
}

double PDDA_EventSet::GetE_SizesMeanSize()
{
    return std::accumulate(_E_Sizes.begin(), _E_Sizes.end(), 0) / double(_E_Sizes.size());
}

double PDDA_EventSet::GetE_RangesMeanRange()
{
    return std::accumulate(_E_Ranges.begin(), _E_Ranges.end(), 0) / double(_E_Ranges.size());
}

void PDDA_EventSet::WriteSerialReadyEventsToCSV()
{
    // Open the output file stream
    std::string filename = "serial_ready_events.csv";
    std::ofstream outFile(filename);

    // Check if the file is open
    if (!outFile.is_open()) {
        throw std::runtime_error("Unable to open file " + filename);
    }

    // Write the header
    outFile << "NumReadyEventsSerial,ReadyEventIndexMeans,ReadyEventIndexStds,ReadyEventNames,E_Sizes\n";

    // Create iterators for each list
    auto itNumReadyEvents = _numReadyEventsSerial.begin();
    auto itReadyEventIndexMeans = _readyEventIndexMeans.begin();
    auto itReadyEventIndexStds = _readyEventIndexStds.begin();
    auto itReadyEventNames = _readyEventNames.begin();
    auto itESizes = _E_Sizes.begin();
    //auto itReadyEventWCTs = _readyEventWCTs.begin();

    // Iterate through the lists and write each row to the CSV file
    while (itNumReadyEvents != _numReadyEventsSerial.end() &&
           itReadyEventIndexMeans != _readyEventIndexMeans.end() &&
           itReadyEventIndexStds != _readyEventIndexStds.end() &&
           itReadyEventNames != _readyEventNames.end() &&
           itESizes != _E_Sizes.end()) {
           //itReadyEventWCTs != _readyEventWCTs.end()) {
        //std::cout << *itReadyEventNames << std::endl;
        outFile << *itNumReadyEvents << ',' << *itReadyEventIndexMeans << ',' << *itReadyEventIndexStds << ',' << *itReadyEventNames << ',' << *itESizes << /*',' << *itReadyEventWCTs <<*/ '\n';

        // Increment the iterators
        ++itNumReadyEvents;
        ++itReadyEventIndexMeans;
        ++itReadyEventIndexStds;
        ++itReadyEventNames;
        ++itESizes;
        //++itReadyEventWCTs;
    }

    // Close the output file stream
    outFile.close();
}

// void PDDA_EventSet::WriteParallelReadyEventsToCSV()
// {
//     // Open the output file stream
//     std::string filename = "parallel_ready_events.csv";
//     std::ofstream outFile(filename);
//
//     // Check if the file is open
//     if (!outFile.is_open()) {
//         throw std::runtime_error("Unable to open file " + filename);
//     }
//
//     // Write the header
//     outFile << "WCT,NumReadyEventsParallel_Arrive,NumReadyEventsParallel_Land,NumReadyEventsParallel_Depart,E_Size\n";
//
//     // Create iterators for each list
//     auto itReadyEventWCTs = _readyEventWCTs.begin();
//     auto itNumReadyEvents = _numReadyByEventType_AllIts.begin();
//     auto itESizes = _E_Sizes.begin();
//
//     // Iterate through the lists and write each row to the CSV file
//     while (itReadyEventWCTs != _readyEventWCTs.end() &&
//            itNumReadyEvents != _numReadyByEventType_AllIts.end() &&
//            itESizes != _E_Sizes.end()) {
//         outFile << *itReadyEventWCTs << ',' << (*itNumReadyEvents).at(0) << ',' << (*itNumReadyEvents).at(1) << ',' << (*itNumReadyEvents).at(2) << ',' << (*itNumReadyEvents).at(3) << ',' << *itESizes << ',' << '\n';
//
//         // Increment the iterators
//         ++itReadyEventWCTs;
//         ++itNumReadyEvents;
//         ++itESizes;
//     }
//
//     // Close the output file stream
//     outFile.close();
// }

void PDDA_EventSet::PrintParallelReadyEventsStats()
{
    int tot_num_ready_arrive_events = std::accumulate(_numReadyArriveEvents.begin(), _numReadyArriveEvents.end(), 0);
    int tot_num_ready_land_events = std::accumulate(_numReadyLandEvents.begin(), _numReadyLandEvents.end(), 0);
    int tot_num_ready_depart_events = std::accumulate(_numReadyDepartEvents.begin(), _numReadyDepartEvents.end(), 0);
    int tot_num_ready_events = tot_num_ready_arrive_events + tot_num_ready_land_events + tot_num_ready_depart_events;

    std::cout << "mean total ready events: " <<  tot_num_ready_events / double(_numReadyArriveEvents.size()) << std::endl;
    std::cout << "mean arrive ready events: " <<  tot_num_ready_arrive_events / double(_numReadyArriveEvents.size()) << std::endl;
    std::cout << "mean land ready events: " <<  tot_num_ready_land_events / double(_numReadyLandEvents.size()) << std::endl;
    std::cout << "mean depart ready events: " <<  tot_num_ready_depart_events / double(_numReadyDepartEvents.size()) << std::endl;

    int tot_num_new_ready_arrive_events = std::accumulate(_numNewReadyArriveEvents.begin(), _numNewReadyArriveEvents.end(), 0);
    int tot_num_new_ready_land_events = std::accumulate(_numNewReadyLandEvents.begin(), _numNewReadyLandEvents.end(), 0);
    int tot_num_new_ready_depart_events = std::accumulate(_numNewReadyDepartEvents.begin(), _numNewReadyDepartEvents.end(), 0);
    int tot_num_new_ready_events = tot_num_new_ready_arrive_events + tot_num_new_ready_land_events + tot_num_new_ready_depart_events;

    std::cout << "mean total new ready events: " <<  tot_num_new_ready_events / double(_numNewReadyArriveEvents.size()) << std::endl;
    std::cout << "mean arrive new ready events: " <<  tot_num_new_ready_arrive_events / double(_numNewReadyArriveEvents.size()) << std::endl;
    std::cout << "mean land new ready events: " <<  tot_num_new_ready_land_events / double(_numNewReadyLandEvents.size()) << std::endl;
    std::cout << "mean depart new ready events: " <<  tot_num_new_ready_depart_events / double(_numNewReadyDepartEvents.size()) << std::endl;

    std::cout << "mean E size: " << std::accumulate(_E_Sizes.begin(), _E_Sizes.end(), 0) / _E_Sizes.size() << std::endl;
}
