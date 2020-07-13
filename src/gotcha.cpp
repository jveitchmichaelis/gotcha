#include <TPHandler.h>
#include <CDensifyParam.h>
#include <Densify.h>
#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char *argv[]){
    if(argc < 4){
        cout << "Expected at least three arguments. Usage:\n    gotcha leftimage rightimage tpoints\n";
        exit(1);
    }

    /* Open left and right images */
    cv::Mat leftimage, rightimage;

    try{
        leftimage = imread(argv[1], IMREAD_GRAYSCALE);
        rightimage = imread(argv[2], IMREAD_GRAYSCALE);

        cout << "Loaded left image: "<< argv[1] << "\n";
        cout << "Loaded right image: "<< argv[2] << "\n";
    }
    catch( exception &e){
        cout << e.what() << '\n';
        exit(1);
    }

    /* Deal with tiepoints */
    TPHandler tph;
    std::vector<CTiePt> tiepoints;
    tph.processTiepointFile(std::string(argv[3]));
    tiepoints = tph.getTiepoints();

    /* Define GOTCHA parameters */
    CDensifyParam params;
    params.m_nProcType = CDensifyParam::GOTCHA;
    params.m_nTPType = CDensifyParam::TP_UNKNOWN;
    params.m_paramGotcha.m_nNeiType = CGOTCHAParam::NEI_8;
    params.m_paramGotcha.m_nMinTile = 256;
    params.m_paramGotcha.m_paramALSC.m_bIntOffset = 1;
    params.m_paramGotcha.m_paramALSC.m_bWeighting = 0;
    params.m_paramGotcha.m_bNeedInitALSC = 0;
    params.m_paramGotcha.m_paramALSC.m_fEigThr = 130;
    params.m_paramGotcha.m_paramALSC.m_nPatch = 12;
    params.m_paramGotcha.m_paramALSC.m_fAffThr = 1.5;
    params.m_paramGotcha.m_paramALSC.m_fDriftThr = 1.0;
    params.m_paramGotcha.m_paramALSC.m_nMaxIter = 8;

    /* Run GOTCHA */
    CDensify densify(params);
    densify.setImages(leftimage, rightimage);
    densify.setTiepoints(tiepoints);
    densify.setOutputFolder(string(argv[4]));
    densify.performDensification();

    return 0;
}
