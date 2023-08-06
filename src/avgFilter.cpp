#include "avgFilter.h"

AvgFilter::AvgFilter(unsigned int windowSize)
{
    // make sure the win is within bounds
    if (windowSize < MAX_WINDOW_SIZE)
    {
        _windowSize = windowSize;
    }
    else if (windowSize < MIN_WINDOW_SIZE)
    {
        _windowSize = MIN_WINDOW_SIZE;
    }
    else
    {
        _windowSize = MAX_WINDOW_SIZE;
    }
    _usedWindow = 0;
}

float AvgFilter::addValue(float value)
{
    if (_usedWindow < _windowSize)
    {
        _usedWindow++;
    }
    shiftValues();
    _values[_usedWindow - 1] = value;
    return avg();
}

void AvgFilter::shiftValues()
{
    for (unsigned int i = 0; i < _usedWindow - 1; i++)
    {
        _values[i] = _values[i + 1];
    }
}

float AvgFilter::avg()
{
    float total = 0.0;
    for (unsigned int i = 0; i < _usedWindow; i++)
    {
        total += _values[i];
    }
    return total / _usedWindow;
}
