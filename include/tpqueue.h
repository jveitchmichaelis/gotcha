#ifndef TPQUEUE_H
#define TPQUEUE_H

#include <CTiePt.h>
#include <queue>
#include <vector>
#include <mutex>

class compareTP
{
public:
  bool operator() (CTiePt tpX, CTiePt tpY) const {
      return (tpX.m_fSimVal > tpY.m_fSimVal);
  }
};

typedef std::priority_queue<CTiePt,std::vector<CTiePt>,compareTP> tiepoint_priority_queue;

class tpqueue
{
public:
    tpqueue();
    void addTiepoint(CTiePt pt);
    CTiePt popTiepoint(void);
    int size(void);
    bool empty(void);
    void clear(void);

private:
    std::mutex queueMutex;
    tiepoint_priority_queue tiepoints;

};

#endif // TPQUEUE_H
