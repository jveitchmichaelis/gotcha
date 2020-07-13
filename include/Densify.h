#ifndef CDENSIFY_H
#define CDENSIFY_H

#include "tpqueue.h"
#include "visitedmap.h"
#include "alscworker.h"
#include "ALSC.h"
#include "CGOTCHAParam.h"
#include "CALSCParam.h"
#include "CDensifyParam.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <timer.h>
#include <thread>

#ifdef _WIN32
#include <direct.h>
#elif defined __linux__
#include <sys/stat.h>
#endif

class CDensify
{

public:
    CDensify();
    CDensify(CDensifyParam params);
    ~CDensify(void);

    int performDensification();
    int getNumTotTps() const {return m_vectpAdded.size();}
    string getResDir(){return m_strResDir;}
    void setTiepoints(std::vector<CTiePt> &tps);
    void setImages(const Mat& imgL, const Mat& imgR);
    void setOutputFolder(string f);
    bool saveMatrix(const Mat& matData, const string strFile);

    static bool compareTP(CTiePt tpX, CTiePt tpY){
        return (tpX.m_fSimVal > tpY.m_fSimVal);
    }

private:
    void filterYDisparity(vector<CTiePt>&);

    vector<CTiePt> getIntToFloatSeed(vector<CTiePt>& vecTPSrc); // get integer Seed point pairs from a float seed point pair

    bool doGotcha(const Mat& matImgL, const Mat& matImgR, vector<CTiePt>& vectpSeeds,
                  const CGOTCHAParam& paramGotcha);
    void subdivideTile(Rect_<float> rectParent, vector< Rect_<float> >& vecRes);
    void makeTiles(vector< Rect_<float> >& vecRectTiles, int nMin);

    void makeDataProducts();
    bool isHavingTP(vector<CTiePt>& vecTpList, CTiePt tp);
    bool saveResult();
    void saveLog();
    void saveResLog(string strFile);
    bool loadTPForDensification(string strTPFile);

    Point2f getTransformed(Point2f ptSrc, bool bToRec, bool bLeft = true);
    void getNeighbourForRectification(const CTiePt tp, vector<CTiePt>& vecNeiTp, const int nNeiType, const Mat& matSim);
    Point2f getDelta(Point2f ptOrg, Point2f ptOffset);

    void saveALSCParam(const CALSCParam& paramALSC, const string strOut);
    void saveGOTCHAParam(CGOTCHAParam& paramGOTCHA, const string strOut);

    double dYlimit = 10;
    int nworkers = 0;
    int retcount = 0;
    vector<ALSCWorker*> workers;

    CDensifyParam m_paramDense;
    Mat m_imgL;
    Mat m_imgR;
    vector<CTiePt> m_vecTPs;

private:
    // inputs
    string m_strResDir; // project name with ending seperator

    // outputs
    vector<CTiePt> m_vectpAdded; // final tp result (i.e. seed tps + newly added tps)

    // Disparity maps:
    // the disparity of a point p_i at x,y in the left image is stored at
    // (x, y) in the disparity maps, e.g., (m_matDisMapX and m_matDisMapY).
    // For example, the position of the corresponding point p'_i in the right image can be defined as
    // (x+xdist, y+y_dist), where 'xdist' and 'ydist' are value at (x,y) in m_matDisMapX and m_matDisMapY, respectively.
    // When disparity values are not known, the vaule of the disparity map is set to zero.
    Mat m_matDisMapX;
    Mat m_matDisMapY;
    Mat m_matDisMapSim; // matching score map: smaller score is better score and -1 indicate unknown

    Mat m_matHL;        // used when the rectified images are used
    Mat m_matHR;

    double m_procTimeWall;
    double m_procTimeCPU;
};

#endif // CDENSIFY_H
