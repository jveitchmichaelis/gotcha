#ifndef ALSCWORKER_H
#define ALSCWORKER_H

#include <ALSC.h>
#include <tpqueue.h>
#include <thread>
#include <visitedmap.h>
#include <opencv2/opencv.hpp>

using namespace std;

class ALSCWorker
{
public:
    ALSCWorker(const Mat &matImgL, const Mat &matImgR, const CALSCParam params, tpqueue* seedlist, visitedMap* vmap, int i);

    void getResults(vector<CTiePt>&);
    void setSeedList(tpqueue*);
    void setVisitedMap(visitedMap*);
    void setID(int);
    void runAsync();
    void run(void);
    std::thread t;

private:
    tpqueue* seedpointList;
    visitedMap* vmap;
    vector<CTiePt> vectpAdded;
    vector<CTiePt> neighbours;
    ALSC matcher;
    int id;
};

#endif // ALSCWORKER_H
