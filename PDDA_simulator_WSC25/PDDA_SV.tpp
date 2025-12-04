#include "PDDA_SV.h"

#include <iostream>

// Define and initialize the static index
template <typename T>
size_t PDDA_SV<T>::_numSVs = 0;

// Constructor: Assigns a unique model index and increments the static index
template <typename T>
PDDA_SV<T>::PDDA_SV(std::string name, T initialValue, T minLimit, T maxLimit) : _name(name), _value(initialValue), _minLimit(minLimit), _maxLimit(maxLimit), _modelIndex(_numSVs++) {}

// Getter for the value
template <typename T>
T PDDA_SV<T>::get() const {
    return _value;
}

// Setter for the value
template <typename T>
void PDDA_SV<T>::set(T newValue) {
    if (newValue > _minLimit && newValue < _maxLimit) _value = newValue;
    else {
        std::cout << _name << " new value " << newValue << " is out-of-bounds!" << std::endl;
        exit(1);
    }
}

// Increment the value
template <typename T>
void PDDA_SV<T>::inc(T incrementBy) {
    if (incrementBy <= 0) {
        std::cout << "Increment value " << incrementBy << " is not a positive value!" << std::endl;
        exit(1);
    }
    if (_value + incrementBy < _maxLimit) _value += incrementBy;
    else {
        std::cout << _name << " new value " << _value + incrementBy << " is greater than the maximum limit!" << std::endl;
        exit(1);
    }
}

// Decrement the value
template <typename T>
void PDDA_SV<T>::dec(T decrementBy) {
    if (decrementBy <= 0) {
        std::cout << "Decrement value " << decrementBy << " is not a positive value!" << std::endl;
        exit(1);
    }
    if (_value - decrementBy > _minLimit) _value -= decrementBy;
    else {
        std::cout << _name << " new value " << _value - decrementBy << " is less than the minimum limit!" << std::endl;
        exit(1);
    }
}

// Get the SV name
template <typename T>
std::string PDDA_SV<T>::getName() const {
    return _name;
}

// Get the model index
template <typename T>
size_t PDDA_SV<T>::getModelIndex() const {
    return _modelIndex;
}
