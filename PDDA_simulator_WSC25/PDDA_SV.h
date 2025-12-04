#pragma once

#include <cstddef>
#include <string>

template <typename T>
class PDDA_SV {
private:
    std::string _name;
    T _value;
    T _minLimit;
    T _maxLimit;
    size_t _modelIndex;
    static size_t _numSVs;
public:
    PDDA_SV(std::string name, T initialValue, T minLimit, T maxLimit);
    T get() const;
    void set(T newValue);
    void inc(T incrementBy = 1);
    void dec(T decrementBy = 1);
    std::string getName() const;
    size_t getModelIndex() const;
};

// Include the implementation file
#include "PDDA_SV.tpp"
