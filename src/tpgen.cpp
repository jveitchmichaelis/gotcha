#include <iostream>
#include <CIPDetector.h>

using namespace std;

int main(int argc, char *argv[]){
    if(argc < 4){
        cout << "Expected at least three arguments. Usage:\n    tpgen leftimage rightimage tiepointfile \n";
        exit(1);
    }

    /* Open left and right images */
    cv::Mat leftimage, rightimage;

    try{
        leftimage = cv::imread(argv[1], IMREAD_GRAYSCALE);
        rightimage = cv::imread(argv[2], IMREAD_GRAYSCALE);

        cout << "Loaded left image: "<< argv[1] << "\n";
        cout << "Loaded right image: "<< argv[2] << "\n";
    }
    catch( exception &e){
        cout << e.what() << '\n';
        exit(1);
    }

    CIPDetectorParam params;

    params.m_nProcType = CIPDetectorParam::SIFT_ONLY;
    params.m_paramSIFT.m_fContThr = 0.005;
    params.m_paramSIFT.m_fDescMatchThr = 0.6;
    params.m_paramSIFT.m_fEdgeThr = 10;
    params.m_paramSIFT.m_nLayers = 3;
    params.m_paramSIFT.m_nOctaves = 4;
    params.m_paramSIFT.m_nStartingOct = 0;

    CIPDetector detector(params);

    /* Output folder */
    detector.setOutputFile(string(argv[3]));
#ifdef _WIN32
    detector.m_strExtEXEPath = "sift.exe";
#else
    detector.m_strExtEXEPath = "./sift";
#endif
    detector.setImages(leftimage, rightimage);
    detector.performDetection();

    return 0;
}
