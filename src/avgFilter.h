#ifndef AVG_FILTER
#define AVG_FILTER

/**
 * @brief the minimum allowed window size
 * of the number of values to store in memory.
 *
 * There's really no point in anything less than 2...
 *
 */
#define MIN_WINDOW_SIZE 2

/**
 * @brief the maximum allowed window size
 * of the number of values to store in memory.
 *
 * The higher the number, the less agressive the
 * affect of the new value will be to the whole.
 *
 * The smaller the number, the quicker you will see
 * change applied, from the new value.
 *
 */
#define MAX_WINDOW_SIZE 10

class AvgFilter
{
private:
    // the total number of window slots available
    unsigned int _windowSize;
    // the number of window slots we have already used
    unsigned int _usedWindow;
    // our window slots
    float _values[MAX_WINDOW_SIZE];

public:
    /**
     * @brief Construct a new Avg Filter object
     *
     * @param windowSize the number of values to store in memory.
     *      The higher the number, the less agressive the
     *      affect of the new value will be to the whole.
     *      The smaller the number, the quicker you will see
     *      change applied, from the new value.
     */
    AvgFilter(unsigned int windowSize);
    /**
     * @brief adds the value provided to the values and returns the new average
     *
     * @param value
     * @return float
     */
    float addValue(float value);
    /**
     * @brief returns the current average of values
     *
     * @return float
     */
    float avg();
    /**
     * @brief shifts the values left, losing the first value (oldest)
     *
     */
    void shiftValues();
};

#endif // AVG_FILTER