#include "tpqueue.h"

tpqueue::tpqueue()
{
}

void tpqueue::addTiepoint(CTiePt tiepoint){
    std::lock_guard<std::mutex> lock(queueMutex);

    tiepoints.push(tiepoint);

    return;
}

void tpqueue::clear(void){
    std::lock_guard<std::mutex> lock(queueMutex);

    tiepoints = tiepoint_priority_queue();

}

CTiePt tpqueue::popTiepoint(){
    std::lock_guard<std::mutex> lock(queueMutex);

    CTiePt tiepoint = tiepoints.top();
    tiepoints.pop();

    return tiepoint;
}

int tpqueue::size(){
    std::lock_guard<std::mutex> lock(queueMutex);
    return tiepoints.size();
}

bool tpqueue::empty(){
    return tiepoints.empty();
}
