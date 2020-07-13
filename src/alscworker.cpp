#include "alscworker.h"

ALSCWorker::ALSCWorker(const Mat &matImgL, const Mat &matImgR, const CALSCParam params, tpqueue* seedlist, visitedMap* vmap, int i)
{
    //ALSC Object for this object to work on
    matcher = ALSC(matImgL, matImgR, params);
    this->vmap = vmap;
    seedpointList = seedlist;
    this->id = i;
}

void ALSCWorker::run()
{
    float pfData[6] = {0, 0, 0, 0, 0, 0};

    // Output tiepoint list, cleared
    vectpAdded.clear();

    // Region Growing Begins
    // While there are seed points left in the tile:

    while (seedpointList->size() > 0) {
        // Pop a tiepoint from the seed heap
        CTiePt tiepoint = seedpointList->popTiepoint();

        // Create neighbours and remove some if necessary
        neighbours.clear();
        vmap->getNeighbours(tiepoint, neighbours);

        // Perform ALSC on neighbour points
        if ((int) neighbours.size() > 0){

            // Initial affine data
            pfData[0] = tiepoint.m_pfAffine[0];
            pfData[1] = tiepoint.m_pfAffine[1];
            pfData[2] = tiepoint.m_pfAffine[2];
            pfData[3] = tiepoint.m_pfAffine[3];
            pfData[4] = tiepoint.m_ptOffset.x;
            pfData[5] = tiepoint.m_ptOffset.y;

            // Perform ALSC on the tiepoints (up to 8)
            matcher.performALSC(&neighbours, pfData);

            // Recover the refined tie points from the ALSC process (those that survived)
            const vector<CTiePt>* matched = matcher.getRefinedTps();

            // If some of the tiepoints were succesfully matched:
            if(matched->size() > 0){
                // Append matched neighbours to the seed point list and update the LUT
                for (int i = 0; i < (int) matched->size(); i++){
                    CTiePt tp = matched->at(i);
                    vmap->addPoint(tp);
                    seedpointList->addTiepoint(tp);
                    vectpAdded.push_back(tp);
                }
            }
        }
    }
    return;
}

void ALSCWorker::getResults(vector<CTiePt>& outputVector){
    outputVector.insert(outputVector.end(), vectpAdded.begin(), vectpAdded.end());
    vectpAdded.clear();
    return;
}

void ALSCWorker::setSeedList(tpqueue* seedlist){
    seedpointList = seedlist;
}

void ALSCWorker::setVisitedMap(visitedMap* vmap){
    this->vmap = vmap;
}

void ALSCWorker::runAsync(){
    t = std::thread(&ALSCWorker::run, this);
}

void ALSCWorker::setID(int i){
    this->id = i;
}
