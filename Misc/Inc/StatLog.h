#ifndef STATLOG_H
#define STATLOG_H
#include <IOSerialStream.h>
#include <string>

class StatLog
{
public:
    StatLog(std::string m_name="");
    float min() const;
    float max() const;
    float stdDev() const;
    float average() const;
    void addValue(float val);
    void reset();
    void print(IOSerialStream * flux, bool printName=true);
    void setLogMinMax(bool log=true);
    void setLogAverage(bool log=true);
    void setLogStdDev(bool log=true);
    void setLogCount(bool log=true);

private:
    std::string m_name; // name of the parameter that is logged
    int m_count; // number of values added since last reset
    float m_min, m_max, m_average, m_sum, m_sumSquare;
    bool m_logMinMax, m_logAverage, m_logCount, m_logStdDev;
};

#endif
