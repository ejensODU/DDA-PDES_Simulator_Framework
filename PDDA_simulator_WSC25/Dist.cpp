#include "Dist.h"

#include <array>

// IntDist implementation
IntDist::IntDist() {}
IntDist::~IntDist() {}

// UniformIntDist implementation 
UniformIntDist::UniformIntDist(int lower, int upper, int distSeed)
    : _lower(lower), _upper(upper)
{
    std::uniform_int_distribution<>::param_type ps(lower, upper);
    _distribution.param(ps);
    
    // Improved seeding mechanism
    std::random_device rd;
	_generator.seed(static_cast<unsigned>(distSeed));
    //_generator.seed(rd() ^ static_cast<unsigned>(distSeed));
    
    // Warm up the generator
    for (int i = 0; i < 10; ++i) {
        _distribution(_generator);
    }

    _ID = "uniform int, lower " + std::to_string(_lower) + ", upper " + std::to_string(_upper);
}

void UniformIntDist::Reset(int distSeed) {
    // Improved reset with better seeding
    std::random_device rd;
    _generator.seed(static_cast<unsigned>(distSeed));
    
    // Warm up after reset
    for (int i = 0; i < 10; ++i) {
        _distribution(_generator);
    }
}

int UniformIntDist::GenRV() { return _distribution(_generator); }

std::string UniformIntDist::getID() { return _ID; }

UniformIntDist::~UniformIntDist() {}

// RealDist implementation
RealDist::RealDist() {}
RealDist::~RealDist() {}

// ConstantRealDist implementation
ConstantRealDist::ConstantRealDist(double value)
    : _value(value)
{
    _ID = "constant, value " + std::to_string(_value);
}

double ConstantRealDist::GenRV() { return _value; }

std::string ConstantRealDist::getID() { return _ID; }

ConstantRealDist::~ConstantRealDist() {}

// NormalDist implementation
NormalDist::NormalDist(double mu, double sigma, int distSeed)
    : _mu(mu), _sigma(sigma)
{
    std::normal_distribution<double>::param_type ps(mu, sigma);
    _distribution.param(ps);
    
    // Improved seeding mechanism
    std::random_device rd;
    _generator.seed(static_cast<unsigned>(distSeed));
    
    // Warm up the generator
    for (int i = 0; i < 10; ++i) {
        _distribution(_generator);
    }

    _ID = "normal, mu " + std::to_string(_mu) + ", sigma " + std::to_string(_sigma);
}

void NormalDist::Reset(int distSeed) {
    // Improved reset with better seeding
    std::random_device rd;
    _generator.seed(static_cast<unsigned>(distSeed));
    
    // Warm up after reset
    for (int i = 0; i < 10; ++i) {
        _distribution(_generator);
    }
}

double NormalDist::GenRV() { return _distribution(_generator); }

std::string NormalDist::getID() { return _ID; }

NormalDist::~NormalDist() {}

// ExpoDist implementation
ExpoDist::ExpoDist(double lambda, int distSeed)
    : _lambda(lambda)
{
    std::exponential_distribution<double>::param_type ps(lambda);
    _distribution.param(ps);
    
    // Improved seeding mechanism
    std::random_device rd;
    _generator.seed(static_cast<unsigned>(distSeed));
    
    // Warm up the generator
    for (int i = 0; i < 10; ++i) {
        _distribution(_generator);
    }

    _ID = "exponential, lambda " + std::to_string(_lambda);
}

void ExpoDist::Reset(int distSeed) {
    // Improved reset with better seeding
    std::random_device rd;
    _generator.seed(static_cast<unsigned>(distSeed));
    
    // Warm up after reset
    for (int i = 0; i < 10; ++i) {
        _distribution(_generator);
    }
}

double ExpoDist::GenRV() { return _distribution(_generator); }

std::string ExpoDist::getID() { return _ID; }

ExpoDist::~ExpoDist() {}

// UniformRealDist implementation
UniformRealDist::UniformRealDist(double lower, double upper, int distSeed)
    : _lower(lower), _upper(upper)
{
    std::uniform_real_distribution<double>::param_type ps(lower, upper);
    _distribution.param(ps);
    
    // Improved seeding mechanism
    std::random_device rd;
    _generator.seed(static_cast<unsigned>(distSeed));
    
    // Warm up the generator
    for (int i = 0; i < 10; ++i) {
        _distribution(_generator);
    }

    _ID = "uniform real, lower " + std::to_string(_lower) + ", upper " + std::to_string(_upper);
}

void UniformRealDist::Reset(int distSeed) {
    // Improved reset with better seeding
    std::random_device rd;
    _generator.seed(static_cast<unsigned>(distSeed));
    
    // Warm up after reset
    for (int i = 0; i < 10; ++i) {
        _distribution(_generator);
    }
}

double UniformRealDist::GenRV() { return _distribution(_generator); }

std::string UniformRealDist::getID() { return _ID; }

UniformRealDist::~UniformRealDist() {}

// TriangularDist implementation
TriangularDist::TriangularDist(double min, double peak, double max, int distSeed)
    : _min(min), _peak(peak), _max(max)
{
    std::array<double, 3> intervals{_min, _peak, _max};
    std::array<double, 3> weights{0, 1, 0};

    std::piecewise_linear_distribution<double>::param_type ps(intervals.begin(), intervals.end(), weights.begin());
    _distribution.param(ps);
    
    // Improved seeding mechanism
    std::random_device rd;
    _generator.seed(static_cast<unsigned>(distSeed));
    
    // Warm up the generator
    for (int i = 0; i < 10; ++i) {
        _distribution(_generator);
    }

    _ID = "triangular, min " + std::to_string(_min) + ", peak " + std::to_string(_peak) + ", max " + std::to_string(_max);
}

void TriangularDist::Reset(int distSeed) {
    // Improved reset with better seeding
    std::random_device rd;
    _generator.seed(static_cast<unsigned>(distSeed));
    
    // Warm up after reset
    for (int i = 0; i < 10; ++i) {
        _distribution(_generator);
    }
}

double TriangularDist::GenRV() { return _distribution(_generator); }

std::string TriangularDist::getID() { return _ID; }

TriangularDist::~TriangularDist() {}