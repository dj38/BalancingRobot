#ifndef STATLOGCOLLECTION_H
#define STATLOGCOLLECTION_H

#include "StatLog.h"
#include "SerialBuffer.h"
#include <vector>

class StatLogCollection
{
public:
    ~StatLogCollection();
    StatLog *appendStatLog(std::string name = "");
    void print(SerialBuffer * flux, bool printName=true);
    void resetAllStatLog();
    void setLogMinMax(bool log=true);
    void setLogAverage(bool log=true);
    void setLogCount(bool log=true);
    void setLogStdDev(bool log=true);
private:
    std::vector<StatLog *> m_list;
};

#endif
