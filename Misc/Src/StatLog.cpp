#include "StatLog.h"
#include <string>
#include <sstream>
#include <cstring>
#include <iostream>
#include "math.h"

using namespace std;

StatLog::StatLog(string name) : m_name (name), m_logMinMax(true), m_logAverage(true), m_logCount(true), m_logStdDev(true)
{
    reset();
}

float StatLog::min() const
{
    if(m_count)
        return(m_min);
    else return(0); // handle NaN
}

float StatLog::max() const
{
    if(m_count)
        return(m_max);
    else return(0); // handle NaN
}

float StatLog::average() const
{
    if(m_count)
        return(m_sum/m_count);
    else return(0); // handle NaN
}

float StatLog::stdDev() const
{
    float avgSquare=average();
    avgSquare*=avgSquare;
    float tmp=m_sumSquare/m_count;
    if(m_count) {
        if(tmp>avgSquare)
            return(sqrt(tmp-avgSquare)); // as per König-Huygens theorem
        else return(0);
    }
    else return(0); // handle NaN
}

void StatLog::addValue(float val)
{
    if(m_logMinMax) {
        if(m_count) {
            if(val > m_max) m_max=val;
            if(val < m_min) m_min=val;
        } else {
            m_min=val;
            m_max=val;
        }
    }
    m_sum+=val;
    m_sumSquare+=(val*val);
    m_count++;
}

void StatLog::reset()
{
    m_sum=0;
    m_count=0;
    m_min=0;
    m_max=0;
    m_sumSquare=0;
}

void StatLog::print(SerialBuffer * flux, bool printName)
{
    if(flux!=0) {
        stringstream ss;
    	if(printName)    ss << m_name << " : ";
        if(m_logAverage) ss << average() << "\t";
        if(m_logMinMax)  ss << m_min << "\t";
        if(m_logMinMax)  ss << m_max << "\t";
        if(m_logStdDev)  ss << stdDev() << "\t";
        if(m_logCount)   ss << m_count;
        ss << "\n";
        flux->write(ss.str());
    }
}


void StatLog::setLogMinMax(bool log)
{
    m_logMinMax=log;
    reset();
}

void StatLog::setLogAverage(bool log)
{
    m_logAverage=log;
    reset();
}

void StatLog::setLogCount(bool log)
{
    m_logCount=log;
    reset();
}

void StatLog::setLogStdDev(bool log)
{
    m_logStdDev=log;
    reset();
}
