#ifndef VISITEDMAP_H
#define VISITEDMAP_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <CTiePt.h>

using namespace std;

class visitedMap
{
public:
    visitedMap(Rect_<float> limagerect = Rect_<float>(), Rect_<float> rimagerect = Rect_<float>());
    void getNeighbours(const CTiePt tiepoint, vector<CTiePt>& neighbours);
    void addPoint(const CTiePt tiepoint);
    void setRectified(Mat m_matHL, Mat m_matHR);

private:
    void checkNeighbours(const CTiePt tp, vector<CTiePt>& vecNeiTp);
    void removeOutsideImage(vector<CTiePt>& tiepoints);
    void removeVisitedPoints(vector<CTiePt>& tiepoints);
    void insert(const CTiePt tiepoint);

    void checkRectifiedNeighbours(const CTiePt tp, vector<CTiePt>& vecNeiTp);
    Point2f getTransformed(Point2f ptSrc, bool bToRec, bool bLeft = true);
    Point2f getDelta(Point2f ptOrg, Point2f ptOffset);

    Mat m_matHL;
    Mat m_matHR;
    bool rectify = false;

    int nWidth;
    bool use8Neighbour;
    std::mutex mapMutex;
    vector<bool> visited;

    Rect_<float> rectImgL;
    Rect_<float> rectImgR;

};

#endif // VISITEDMAP_H
