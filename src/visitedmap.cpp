#include "visitedmap.h"

visitedMap::visitedMap(Rect_<float> rectImgL, Rect_<float> rectImgR)
{
    this->rectImgL = rectImgL;
    this->rectImgR = rectImgR;
    visited = vector<bool>(rectImgR.area(), false);
    nWidth = rectImgL.width;
    use8Neighbour = true;
}

void visitedMap::getNeighbours(const CTiePt tiepoint, vector<CTiePt>& neighbours){
    // Lock
    std::lock_guard<std::mutex> lock(mapMutex);

    // Get neighbours of tiepoint
    if(rectify) checkRectifiedNeighbours(tiepoint, neighbours);
    checkNeighbours(tiepoint, neighbours);

    // Check which tiepoints are within either image
    removeOutsideImage(neighbours);

    // Remove those which have already been visited
    removeVisitedPoints(neighbours);

    // Update map of visited pixels to include the new neighbours
    for(int i = 0; i < (int) neighbours.size(); i++){
        insert(neighbours.at(i));
    }
}

void visitedMap::insert(const CTiePt tiepoint){
    int nX = (int)floor(tiepoint.m_ptL.x);
    int nY = (int)floor(tiepoint.m_ptL.y);

    int nIdx = nY * nWidth + nX;

    visited.at(nIdx) = true;
}

void visitedMap::addPoint(const CTiePt tiepoint){
    std::lock_guard<std::mutex> lock(mapMutex);
    insert(tiepoint);
}


void visitedMap::removeOutsideImage(vector<CTiePt>& tiepoints){
    // Remove any tie points that are outside either image.

    vector<CTiePt>::iterator iter;

    for (iter = tiepoints.begin(); iter < tiepoints.end(); ){
        if (!rectImgL.contains(iter->m_ptL) || !rectImgR.contains(iter->m_ptR))
            tiepoints.erase(iter);
       else
            iter++;
    }

}

void visitedMap::removeVisitedPoints(vector<CTiePt>& tiepoints){
    // This function removes points from a tiepoint list if they've already been processed

    vector<CTiePt>::iterator iter;

    for (iter = tiepoints.begin(); iter < tiepoints.end();){
        int nX = (int)floor(iter->m_ptL.x);
        int nY = (int)floor(iter->m_ptL.y);

        int nIdx = nY * nWidth + nX;

        if (visited.at(nIdx)){
            tiepoints.erase(iter);
        }
        else
            iter++;
    }
}

void visitedMap::checkNeighbours(const CTiePt tp, vector<CTiePt>& vecNeiTp){

    Point2f ptLeft = tp.m_ptL;
    Point2f ptRight = tp.m_ptR;

    vecNeiTp.push_back(tp);

    CTiePt tp2;
    tp2.m_ptL = ptLeft + Point2f(0,1);
    tp2.m_ptR = ptRight + Point2f(0,1);
    vecNeiTp.push_back(tp2);
    tp2.m_ptL = ptLeft + Point2f(0,-1);
    tp2.m_ptR = ptRight + Point2f(0,-1);
    vecNeiTp.push_back(tp2);

    tp2.m_ptL = ptLeft + Point2f(1,0);
    tp2.m_ptR = ptRight + Point2f(1,0);
    vecNeiTp.push_back(tp2);
    tp2.m_ptL = ptLeft + Point2f(-1,0);
    tp2.m_ptR = ptRight + Point2f(-1,0);
    vecNeiTp.push_back(tp2);

    if (use8Neighbour){
        CTiePt tp8;
        tp8.m_ptL = ptLeft + Point2f(-1,-1);
        tp8.m_ptR = ptRight + Point2f(-1,-1);
        vecNeiTp.push_back(tp8);
        tp8.m_ptL = ptLeft + Point2f(1,1);
        tp8.m_ptR = ptRight + Point2f(1,1);
        vecNeiTp.push_back(tp8);
        tp8.m_ptL = ptLeft + Point2f(-1,1);
        tp8.m_ptR = ptRight + Point2f(-1,1);
        vecNeiTp.push_back(tp8);
        tp8.m_ptL = ptLeft + Point2f(1,-1);
        tp8.m_ptR = ptRight + Point2f(1,-1);
        vecNeiTp.push_back(tp8);
    }
}

void visitedMap::setRectified(Mat m_matHL, Mat m_matH){
    rectify = true;
    this->m_matHL = m_matHL;
    this->m_matHR = m_matHR;
}

void visitedMap::checkRectifiedNeighbours(const CTiePt tp, vector<CTiePt>& vecNeiTp){
    Point2f ptLeft = tp.m_ptL;
    Point2f ptRight = tp.m_ptR;

    // Neighbours above and below
    CTiePt tp2;
    Point2f ptOff = getDelta( getTransformed(ptLeft, false), Point2f(0, 1) );
    tp2.m_ptL = ptLeft + ptOff; //Point2f(0,ptOffset.y);
    tp2.m_ptR = ptRight + ptOff; //Point2f(0,ptOffset.y);
    vecNeiTp.push_back(tp2);

    ptOff = getDelta( getTransformed(ptLeft, false), Point2f(0, -1) );
    tp2.m_ptL = ptLeft + ptOff; //Point2f(0,-ptOffset.y);
    tp2.m_ptR = ptRight + ptOff; //Point2f(0,-ptOffset.y);
    vecNeiTp.push_back(tp2);

    // Neighbours left and right
    ptOff = getDelta( getTransformed(ptLeft, false), Point2f(1, 0) );
    tp2.m_ptL = ptLeft + ptOff; //Point2f(ptOffset.x,0);
    tp2.m_ptR = ptRight + ptOff; //Point2f(ptOffset.x,0);
    vecNeiTp.push_back(tp2);

    ptOff = getDelta( getTransformed(ptLeft, false), Point2f(-1, 0) );
    tp2.m_ptL = ptLeft + ptOff;//Point2f(-ptOffset.x,0);
    tp2.m_ptR = ptRight + ptOff;//Point2f(-ptOffset.x,0);
    vecNeiTp.push_back(tp2);

    // Diagonal neighbours
    if (use8Neighbour){
        CTiePt tp8;

        Point2f ptOff = getDelta( getTransformed(ptLeft, false), Point2f(-1, -1) );
        tp8.m_ptL = ptLeft - ptOff; //+ Point2f(-ptOffset.x,-);
        tp8.m_ptR = ptRight - ptOff;// + Point2f(-1,-1);
        vecNeiTp.push_back(tp8);

        ptOff = getDelta( getTransformed(ptLeft, false), Point2f(1, 1) );
        tp8.m_ptL = ptLeft + ptOff;//Point2f(1,1);
        tp8.m_ptR = ptRight + ptOff;//Point2f(1,1);
        vecNeiTp.push_back(tp8);

        ptOff = getDelta( getTransformed(ptLeft, false), Point2f(-1, 1) );
        tp8.m_ptL = ptLeft + ptOff;//Point2f(-ptOffset.x, ptOffset.y);
        tp8.m_ptR = ptRight + ptOff;//Point2f(-ptOffset.x, ptOffset.y);
        vecNeiTp.push_back(tp8);

        ptOff = getDelta( getTransformed(ptLeft, false), Point2f(1, -1) );
        tp8.m_ptL = ptLeft + ptOff; //Point2f(ptOffset.x,-ptOffset.y);
        tp8.m_ptR = ptRight + ptOff; //Point2f(ptOffset.x,-ptOffset.y);
        vecNeiTp.push_back(tp8);
    }
}

Point2f visitedMap::getTransformed(Point2f ptSrc, bool bToRec, bool bLeft){
    Point2f ptRes;

    // A transform which converts a point in the left original coordinate to the left rectified coordinate
    Mat matTr;
    if(bToRec && bLeft) matTr = m_matHL;
    else if (bToRec && !bLeft) matTr = m_matHR;
    else if (!bToRec && bLeft) matTr = m_matHL.inv();
    else if (!bToRec && !bLeft)matTr = m_matHR.inv();

    // The x/y point in homogenous coordinates (x,y,w)
    Mat matPt(3,1, CV_32F);
    matPt.at<float>(0,0) =  ptSrc.x;
    matPt.at<float>(1,0) =  ptSrc.y;
    matPt.at<float>(2,0) =  1.f;

    // Perform the transformation and then move back to (x,y) by dividing by w.
    matPt = matTr * matPt;
    matPt = matPt/matPt.at<float>(2,0);
    ptRes = Point2f(matPt.at<float>(0,0), matPt.at<float>(1,0));

    return ptRes;
}

Point2f visitedMap::getDelta(Point2f ptOrg, Point2f ptOffset){
    // inputs are in original image
    // outputs are in rectified coordinate
    Point2f ptRes;

    Point ptNei = ptOrg + ptOffset;
    ptRes = getTransformed(ptNei, true) - getTransformed(ptOrg, true);

    return ptRes;
}

