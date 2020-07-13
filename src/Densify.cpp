#include "Densify.h"

using namespace std;

CDensify::CDensify(){
    return;
}

CDensify::CDensify(CDensifyParam params)
{
    m_paramDense = params;
}

void CDensify::setImages(const Mat &imgL,const Mat &imgR){
    m_imgL = imgL;
    m_imgR = imgR;
}

void CDensify::setTiepoints(std::vector<CTiePt> &tps){
    m_vecTPs = tps;
}

Point2f CDensify::getTransformed(Point2f ptSrc, bool bToRec, bool bLeft){
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

int CDensify::performDensification(){
    vector<CTiePt> vecSeedTemp = getIntToFloatSeed(m_vecTPs);

    if (!doGotcha(m_imgL, m_imgR, vecSeedTemp, m_paramDense.m_paramGotcha))
        return CDensifyParam::GOTCHA_ERR;

    cout << "Finished GOTCHA\n";

    // remove TP with large y disparity
    // nb. sometimes large y disparity can be produced even from a rectified MER pair
    // (especially this is happening in a place near to a Rover)

    cout << "Removing points with large y-disparity\n";
    //filterYDisparity(m_vectpAdded);
    cout << "After processing, " << m_vectpAdded.size() << " points matched.\n";

    cout << "Making data products\n";
    makeDataProducts();

    cout << "Saving results and log\n";
    saveResult();
    saveLog();

    return CDensifyParam::NO_ERR;
}

vector<CTiePt> CDensify::getIntToFloatSeed(vector<CTiePt>& vecTPSrc) {
    vector<CTiePt> vecRes;

    int nLen = vecTPSrc.size();

    cout << nLen << endl;

    for (int i = 0; i < nLen; i++){

        //get four neighbours
        CTiePt tp = vecTPSrc.at(i);
        Point2f ptL = tp.m_ptL;
        Point2f ptR = tp.m_ptR;
        float dX = 0, dY = 0; // xy offset to make integer seed

        // it could be trickier if the input is from Rectified folder
        if (m_paramDense.m_nTPType == CDensifyParam::TP_RECTIFIED){
            // 1. transform back to the original coordinate system from the rectified
            Point2f ptLOrg = getTransformed(ptL, false);

            // 2. get offset to make integer in original coordinate
            dX = floor(ptLOrg.x) - ptLOrg.x;
            dY = floor(ptLOrg.y) - ptLOrg.y;
            if (dX == 0 && dY == 0){
                vecRes.push_back(tp);
                continue;
            }
            else {
                //3. get the offset in the rectified coordinate
                Point2f ptRes = getDelta(ptLOrg, Point2f(dX, dY));
                dX = ptRes.x;
                dY = ptRes.y;
            }
        }
        else{
            dX = floor(ptL.x) - ptL.x;
            dY = floor(ptL.y) - ptL.y;
            if (dX == 0 && dY == 0){
                vecRes.push_back(tp);
                continue;
            }
        }

        Point2f ptDelta(dX,dY);
        Point2f ptIntL = ptL + ptDelta; // ptIntL may be in float if ptL is came from the Rectified coordinate
        Point2f ptIntR = ptR + ptDelta;

        /////////////////////////////////////////////////////
        // collect 4 integer seed and validate
        /////////////////////////////////////////////////////
        CTiePt tpTemp = tp;
        vector<CTiePt> vectpSeeds;
        // pt1 (top-left)
        tpTemp.m_ptL = ptIntL;
        tpTemp.m_ptR = ptIntR;
        vectpSeeds.push_back(tpTemp);
        // pt2 (top-right)
        if (m_paramDense.m_nTPType == CDensifyParam::TP_RECTIFIED){
            Point2f ptOff = getDelta( getTransformed(ptIntL, false), Point2f(1, 0) );
            tpTemp.m_ptL = ptIntL + ptOff;
            tpTemp.m_ptR = ptIntR + ptOff;
        } else{
            tpTemp.m_ptL = ptIntL + Point2f(1, 0);
            tpTemp.m_ptR = ptIntR + Point2f(1, 0);
        }
        vectpSeeds.push_back(tpTemp);
        // pt3 (bottom-left)
        if (m_paramDense.m_nTPType == CDensifyParam::TP_RECTIFIED){
            Point2f ptOff = getDelta( getTransformed(ptIntL, false), Point2f(0, 1) );
            tpTemp.m_ptL = ptIntL + ptOff;
            tpTemp.m_ptR = ptIntR + ptOff;
        }else{
            tpTemp.m_ptL = ptIntL + Point2f(0, 1);
            tpTemp.m_ptR = ptIntR + Point2f(0, 1);
        }
        vectpSeeds.push_back(tpTemp);
        // pt 4 (bottom-right)
        if (m_paramDense.m_nTPType == CDensifyParam::TP_RECTIFIED){
            Point2f ptOff = getDelta( getTransformed(ptIntL, false), Point2f(1, 1) );
            tpTemp.m_ptL = ptIntL + ptOff;
            tpTemp.m_ptR = ptIntR + ptOff;
        }else{
            tpTemp.m_ptL = ptIntL + Point2f(1, 1);
            tpTemp.m_ptR = ptIntR + Point2f(1, 1);
        }
        vectpSeeds.push_back(tpTemp);

        //Apply ALSC on initial neighbouring seed points are good
        ALSC alsc(m_imgL, m_imgR,  m_paramDense.m_paramGotcha.m_paramALSC);
        alsc.performALSC(&vectpSeeds);
        vectpSeeds.clear();
        alsc.getRefinedTps(vectpSeeds); // hard-copy

        for (int j = 0; j < (int)vectpSeeds.size(); j++){
            CTiePt tpRef = vectpSeeds.at(j);
            if (tpRef.m_fSimVal != CTiePt::NOT_DEF){
                vecRes.push_back(tpRef);
            }
        }
    }

    return vecRes;
}

Point2f CDensify::getDelta(Point2f ptOrg, Point2f ptOffset){
    // inputs are in original image
    // outputs are in rectified coordinate
    Point2f ptRes;

    Point ptNei = ptOrg + ptOffset;
    ptRes = getTransformed(ptNei, true) - getTransformed(ptOrg, true);

    return ptRes;
}

void CDensify::saveLog(){

    cout << "Saving log file\n";

    string strFile = m_strResDir + "procLog.txt";

    // save parameter log
    saveGOTCHAParam(m_paramDense.m_paramGotcha, strFile);
    saveALSCParam(m_paramDense.m_paramGotcha.m_paramALSC, strFile);
    saveResLog(strFile);

    return;
}

void CDensify::saveALSCParam(const CALSCParam& paramALSC, const string strOut){
    ofstream sfLog;
    sfLog.open(strOut.c_str(), ios::app | ios::out);

    if (sfLog.is_open()){

        sfLog << "<ALSC parameters>" << endl;
        sfLog << "The size of a matching patch: " << paramALSC.m_nPatch << endl;
        sfLog << "Maximum eigenval: " << paramALSC.m_fEigThr << endl;
        sfLog << "Maximum iteration: " << paramALSC.m_nMaxIter << endl;
        sfLog << "Affine distortion limit: " << paramALSC.m_fAffThr << endl;
        sfLog << "Translation limit: " << paramALSC.m_fDriftThr << endl;
        sfLog << "Use intensity offset parameter: " << paramALSC.m_bIntOffset << endl;
        sfLog << "Use weighting coefficients: " << paramALSC.m_bWeighting << endl;
        sfLog << endl;

        sfLog.close();
    }
}

void CDensify::saveGOTCHAParam(CGOTCHAParam& paramGOTCHA, const string strOut){
    ofstream sfLog;
    sfLog.open(strOut.c_str(), ios::app | ios::out);

    if (sfLog.is_open()){

        sfLog << "<GOTCHA parameters>" << endl;
        sfLog << "Neighbour type: " << paramGOTCHA.getNeiType()<< endl;
        sfLog << "Diffusion iteration: " << paramGOTCHA.m_nDiffIter << endl;
        sfLog << "Diffusion threshold: " << paramGOTCHA.m_fDiffThr << endl;
        sfLog << "Diffusion coefficient: " << paramGOTCHA.m_fDiffCoef << endl;
        sfLog << "Minimum image tile size: " <<  paramGOTCHA.m_nMinTile << endl;
        sfLog << "Need initial ALSC on seed TPs: " << paramGOTCHA.m_bNeedInitALSC << endl;
        sfLog << endl;

        sfLog.close();
    }
}

void CDensify::saveResLog(string strFile){

    ofstream sfLog;
    sfLog.open(strFile.c_str(), ios::app | ios::out);

    if (sfLog.is_open()){
        sfLog << "<Processing results>" << endl;
        sfLog << "Processing method: GOTCHA" << endl;
        sfLog << "Total number of seed TPs: " << m_vecTPs.size() << endl;
        sfLog << "Total number of final TPs: " << m_vectpAdded.size() << endl;
        sfLog << "Total wall processing time [sec]: " << m_procTimeWall << endl;
        sfLog << "Total CPU processing time [sec]: " << m_procTimeCPU << endl;
        sfLog.close();
    }
    return;
}

bool CDensify::saveMatrix(const Mat& matData, const string strFile){

    ofstream sfOut;
    sfOut.open(strFile.c_str());

    if (sfOut.is_open()){

        sfOut << matData.rows << " " << matData.cols << endl;

        for (int i = 0; i < matData.rows; i++){
            for (int j = 0; j < matData.cols; j++){
                if (matData.depth() == CV_32F)
                    sfOut << matData.at<float>(i,j) << " ";
                else if (matData.depth() == CV_64F)
                    sfOut << matData.at<double>(i,j) << " ";
                else if (matData.depth() == CV_8U)
                    sfOut << matData.at<uchar>(i,j) << " ";
            }
            sfOut << endl;
        }
        sfOut.close();
    }
    else{
        cout << "Couldn't open file " << strFile;
        return false;
    }

    return true;
}

void CDensify::makeDataProducts(){

    /////////////////////////////////////////////////////////////////////////////////////////////
    // prepare output data product from the list of desified tiepoints
    if (m_paramDense.m_nTPType == CDensifyParam::TP_RECTIFIED){
        // As the estimated TPs are in the rectified image it should be transformed
        // back to the original coordinate system again
        int nLen = m_vectpAdded.size();
        for (int nIdx = 0 ; nIdx < nLen; nIdx++){
            Point2f ptL = getTransformed(m_vectpAdded.at(nIdx).m_ptL, false);        // inverse transform of left
            // make sure it is integer as there might be roundoff error
            m_vectpAdded.at(nIdx).m_ptL.x = round(ptL.x);
            m_vectpAdded.at(nIdx).m_ptL.y = round(ptL.y);
            m_vectpAdded.at(nIdx).m_ptR = getTransformed(m_vectpAdded.at(nIdx).m_ptR, false, false); // inverse tr of right

        }
    }

    // make output products (disparity map x,y and sim)
    int nW = m_imgL.cols;
    int nH = m_imgL.rows;

    // crear&initialise output buffers
    m_matDisMapX = Mat::zeros(nH, nW, CV_32FC1);
    m_matDisMapY = Mat::zeros(nH, nW, CV_32FC1);
    m_matDisMapSim = Mat::ones(nH, nW, CV_32FC1)*-1;

    // fill the disparity map
    int nLen = m_vectpAdded.size();
    for (int i = 0 ; i < nLen; i++){
        CTiePt tp = m_vectpAdded.at(i);
        Point2f ptL = tp.m_ptL;
        Point2f ptR = tp.m_ptR;
        float fSim = tp.m_fSimVal;

        int x = (int) ptL.x;
        int y = (int) ptL.y;
        Rect_<int> rect(0,0,nW,nH);
        if (rect.contains(Point2i(x,y))) {
            m_matDisMapX.at<float>(y,x) = ptR.x - x;
            m_matDisMapY.at<float>(y,x) = ptR.y - y;
            m_matDisMapSim.at<float>(y,x) = fSim;
        }
    }
}

void CDensify::setOutputFolder(string f){
    m_strResDir = f;
    if(m_strResDir[m_strResDir.length()-1] != '/'){
        m_strResDir += '/';
    }

#ifdef _WIN32
    _mkdir(m_strResDir.c_str());
#else
    mkdir(m_strResDir.c_str(), ACCESSPERMS);
#endif
}

bool CDensify::saveResult(){

    string strFile;

    cout << "Saving results\n";

    strFile = m_strResDir + "disMapX.txt";
    saveMatrix(m_matDisMapX, strFile);

    strFile = m_strResDir + "disMapY.txt";
    saveMatrix(m_matDisMapY, strFile);

    strFile = m_strResDir + "disparity_sim.txt";
    saveMatrix(m_matDisMapSim, strFile);

    // Normalise disparity maps

    double min_disp, max_disp;
    cv::minMaxIdx(m_matDisMapX, &min_disp, &max_disp);
    cv::Mat norm_x_disparity = 255*(m_matDisMapX - min_disp)/(max_disp-min_disp);
    norm_x_disparity.convertTo(norm_x_disparity, CV_8UC1);

    cv::Mat colourmapped_output;
    cv::applyColorMap(norm_x_disparity, colourmapped_output, cv::COLORMAP_MAGMA);
    cv::imwrite(m_strResDir + "disparity_x.png", colourmapped_output);

    cv::minMaxIdx(m_matDisMapY, &min_disp, &max_disp);
    cv::Mat norm_y_disparity  = 255*(m_matDisMapY - min_disp)/(max_disp-min_disp);
    norm_y_disparity.convertTo(norm_y_disparity, CV_8UC1);
    cv::applyColorMap(norm_y_disparity, colourmapped_output, cv::COLORMAP_MAGMA);
    cv::imwrite(m_strResDir +"disparity_y.png", colourmapped_output);

    cv::minMaxIdx(m_matDisMapSim, &min_disp, &max_disp);
    cv::Mat norm_conf_disparity = 255*(m_matDisMapSim - min_disp)/(max_disp-min_disp);
    norm_conf_disparity.convertTo(norm_conf_disparity, CV_8UC1);
    cv::applyColorMap(norm_conf_disparity, colourmapped_output, cv::COLORMAP_MAGMA);
    cv::imwrite(m_strResDir + "disparity_conf.png", colourmapped_output);

    return true;
}

void CDensify::makeTiles(vector< Rect_<float> >& vecRectTiles, int nMinPatchSize){

    // Generate a list of tiles for an image.  Each tile has is at least nMinPatchSize square.

    vector< Rect_<float> >::iterator iter;

    for (iter = vecRectTiles.begin(); iter < vecRectTiles.end(); ){
        Rect_<float> rectParent = *iter;

        // If the width of the current tile is larger than twice the patch size in both directions,
        // split it into four.  Otherwise we can't break it down any further.
        if (rectParent.width > 2*nMinPatchSize && rectParent.height > 2*nMinPatchSize){

            // Remove the 'parent' tile
            vecRectTiles.erase(iter);

            // Split into four
            vector< Rect_<float> > vecSubTiles;
            subdivideTile(rectParent, vecSubTiles);

            // Add the children to the tile list
            vecRectTiles.insert(vecRectTiles.end(), vecSubTiles.begin(),  vecSubTiles.end());

            // Begin again with the new tiles added.  This would make a nice recursive function.. but we'll ignore that
            iter = vecRectTiles.begin();
        }
        else
            iter++;
    }
}

void CDensify::subdivideTile(Rect_<float> rectParent, vector< Rect_<float> >& vecRes){

    float fHalfW = rectParent.width/2.f;
    float fHalfH = rectParent.height/2.f;

    // Note: top left corner is (0,0) in graphics coords
    Point2f ptTopLeft[4];
    ptTopLeft[0] = Point2f(rectParent.x, rectParent.y);
    ptTopLeft[1] = Point2f(rectParent.x+fHalfW, rectParent.y);
    ptTopLeft[2] = Point2f(rectParent.x, rectParent.y+fHalfH);
    ptTopLeft[3] = Point2f(rectParent.x+fHalfW, rectParent.y+fHalfH);

    // Tile Size
    Size sz(fHalfW, fHalfH);

    // Upper left tile
    Rect_<float> subTile(ptTopLeft[0], sz);
    vecRes.push_back(subTile);

    // Upper right tile
    subTile = Rect(ptTopLeft[1], sz);
    vecRes.push_back(subTile);

    // Lower left tile
    subTile = Rect(ptTopLeft[2], sz);
    vecRes.push_back(subTile);

    // Lower right tile
    subTile = Rect(ptTopLeft[3], sz);
    vecRes.push_back(subTile);

}

bool CDensify::doGotcha(const Mat& matImgL, const Mat& matImgR, vector<CTiePt>& vectpSeeds,
                        const CGOTCHAParam& paramGotcha){


    bool bRes = true;
    bool dotiled = false;
    cout << "Running GOTCHA\n";
    ////////////////////////////////////////////
    // Parameter preparation
    Mat matSimMap = Mat::ones(matImgL.rows, matImgL.cols, CV_32FC1); // 2D similarity map for diffusion
    matSimMap = matSimMap*-1;
    tpqueue seedpointList;

    Rect_<float> rectImgR = Rect_<float>(0, 0, matImgR.cols, matImgR.rows);
    Rect_<float> rectImgL = Rect_<float>(0, 0, matImgL.cols, matImgL.rows);

    // Initial ALSC seed refinement
    if (paramGotcha.m_bNeedInitALSC){
        cout << "Refining seeds\n";
        ALSC alsc(matImgL, matImgR, paramGotcha.m_paramALSC);
        alsc.performALSC(&vectpSeeds);
        vectpSeeds.clear();
        alsc.getRefinedTps(vectpSeeds); // hard-copy

    }

    //filterYDisparity(vectpSeeds);
    visitedMap vmap(rectImgL, rectImgR);
    if(m_paramDense.m_nTPType == CDensifyParam::TP_RECTIFIED){
        vmap.setRectified(m_matHL, m_matHR);
    }


    // Initialise similarity map with seedpoints
    for (int i = 0; i < (int)vectpSeeds.size(); i++){
        int nX =  (int)floor(vectpSeeds.at(i).m_ptL.x);
        int nY =  (int)floor(vectpSeeds.at(i).m_ptL.y);
        matSimMap.at<float>(nY, nX) = vectpSeeds.at(i).m_fSimVal;

        // Also update the lookup table to let us know that these points were processed
        //vmap.addPoint(vectpSeeds.at(i));
        seedpointList.addTiepoint(vectpSeeds.at(i));
    }

    m_vectpAdded.clear();

    double t1_wall = get_wall_time();
    double t1_cpu = get_cpu_time();

    if(dotiled){
        // Tile the image, pass in the entire image as the first tile which will be subdivided
        vector< Rect_<float> > vecRectTiles;
        vecRectTiles.push_back(Rect(0., 0., matImgL.cols, matImgL.rows));
        makeTiles(vecRectTiles, paramGotcha.m_nMinTile);
        cout << "Image split into" << (int)vecRectTiles.size() << "tiles\n";

        for(int i = 0; i < (int) vecRectTiles.size(); i++){
            seedpointList.clear();
            for (int j = 0; j < (int) vectpSeeds.size(); j++){
                CTiePt tp = vectpSeeds.at(j);
                if (vecRectTiles.at(i).contains(tp.m_ptL))
                    seedpointList.addTiepoint(tp);
            }
            ALSCWorker worker(matImgL, matImgR, paramGotcha.m_paramALSC, &seedpointList, &vmap, i);
            worker.setSeedList(&seedpointList);
            worker.setVisitedMap(&vmap);
            worker.run();
            worker.getResults(m_vectpAdded);
        }

    }else{
        nworkers = std::thread::hardware_concurrency();
        workers.clear();

        cout << "Spawning " << nworkers <<  " threads.\n";

        for(int i=0; i < nworkers; i++){
            ALSCWorker* worker = new ALSCWorker(matImgL, matImgR, paramGotcha.m_paramALSC, &seedpointList, &vmap, i);
            workers.push_back(worker);
            worker->runAsync();
        }

        for(int i=0; i < nworkers; i++){
            workers.at(i)->t.join();
            workers.at(i)->getResults(m_vectpAdded);
        }
    }

    double t2_wall = get_wall_time();
    double t2_cpu = get_cpu_time();

    m_procTimeWall = t2_wall- t1_wall;
    m_procTimeCPU = t2_cpu - t1_cpu;

    cout << "Finished processing: " << m_vectpAdded.size() << " points were added in "<< m_procTimeWall << "s\n";
    cout << "CPU time " << m_procTimeCPU << "s\n";

    return bRes;
}

void CDensify::filterYDisparity(vector<CTiePt>& points){
    auto new_end = remove_if(points.begin(), points.end(), [this](const CTiePt point){
                    return (fabs(point.m_ptL.y - point.m_ptR.y) > dYlimit);
                }
              );

    points.erase(new_end, points.end());
}

CDensify::~CDensify(void){
    for(int i=0; i<nworkers;i++){
        delete(workers.at(i));
    }
}
