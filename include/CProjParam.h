#ifndef CPROJPARAM_H
#define CPROJPARAM_H

// directory names used
#define DIR_FEATURE "Features"
#define DIR_RECTIFY "Rectify"
#define DIR_IMAGES "Images"
#define DIR_CALI "Calibration"
#define DIR_DENSE "Dense"
#define DIR_TRI "Tri"
#define DIR_POST_REF "PostRef"
#define DIR_RESOURCE "Resources"
#define DIR_PROJ_HEADING "Proj_"
#define DIR_PDS "PDSdata"
#define DIR_PDS_LEFT "PDS_L"
#define DIR_PDS_RIGHT "PDS_R"
#define DIR_MERGED_TRI "MergedTri"
#define DIR_IMG_NET "ImgNet"
#define DIR_INTER_STEREO "InterStereo"
#define DIR_BA "BA"
#define DIR_DTM "DTM"

// claibration file name
#define FILE_CALI_L "camL.txt"
#define FILE_CALI_R "camR.txt"

// file names used for feature dectection
#define FILE_SIFT_FRAME_L "SIFTFrameL.txt"
#define FILE_SIFT_FRAME_R "SIFTFrameR.txt"
#define FILE_SURF_FRAME_L "SURFrameL.txt"
#define FILE_SURF_FRAME_R "SURFrameR.txt"
#define FILE_TP_SIFT "TP_SIFT.txt"
#define FILE_TP_SURF "TP_SURF.txt"
//#define FILE_TP_SIFT_ALSC "TP_SIFT_ALSC.txt"
//#define FILE_TP_SURF_ALSC "TP_SURF_ALSC.txt"
//#define FILE_TP_DOG_ALSC "TP_DOG_ALSC.txt"
//#define FILE_TP_DOH_ALSC "TP_DOH_ALSC.txt"
//#define FILE_TP_CSA "TP_CSA.txt"
#define FILE_TP_SIFT_CSA "TP_SIFT_CSA.txt"
#define FILE_TP_SURF_CSA "TP_SURF_CSA.txt"
#define FILE_TP_DOG_CSA "TP_DOG_CSA.txt"
#define FILE_TP_DOH_CSA "TP_DOH_CSA.txt"

// file names used for rectify
#define FILE_IMG_RECT_L "rectImgL.jpg"
#define FILE_IMG_RECT_R "rectImgR.jpg"
#define FILE_F "F.txt"
#define FILE_H_R "HR.txt"
#define FILE_H_L "HL.txt"
#define FILE_TP_RECT "TP_RECT.txt"
#define FILE_TP_INLIER_IDX "TP_InlierIdx.txt"

// file names in densification
#define FILE_TP_DENSE "TP_DENSE.txt"
#define FILE_DIS_MAP_X "disMapX.txt"
#define FILE_DIS_MAP_Y "disMapY.txt"
#define FILE_DIS_MAP_SIM "sim.txt"
#define FILE_TP_DENSE_REF "TP_DENSE_r.txt" // outlier removal using RANSAC
#define FILE_DIS_MAP_X_REF "disMapX_r.txt"
#define FILE_DIS_MAP_Y_REF "disMapY_r.txt"
#define FILE_DIS_MAP_SIM_REF "sim_r.txt"
//#define FILE_DIS_MAP_L "TpL.txt"  // tiepoints as a map
//#define FILE_DIS_MAP_R "TpR.txt"

// Triangulation
#define FILE_TRI_TP_TEMP "tempTp.txt";
#define FILE_TRI_EDGE_LIST "triEdge.txt"
#define FILE_TRI_PLY "rec.ply"
//#define FILE_TRI_PT_LIST "3dPtList.txt"

// refined (this will be obsolete. used in old BA used in GUI I/F)
#define FILE_CALI_L_UPDATED "camLupdated.txt"
#define FILE_CALI_R_UPDATED "camRupdated.txt"
#define FILE_PLY_UPDATED "recRef.ply"
//#define FILE_BA_XYZ "updatedXYZ.txt"
//#define FILE_PT_LIST_UPDATED "3dPtList.txt"

// image network
#define FILE_IMG_NET "imgNet.txt"
#define FILE_PROJECTION "projectionForBA.txt"

// Bundle adjustment
#define FILE_MOT_INIT_4BA "initialRandT4BA.txt"
#define FILE_CAM_INIT_4BA "initialCameraParam4BA.txt"
#define FILE_R_AFTER_BA "updatedR.txt"
#define FILE_T_AFTER_BA "updatedT.txt"
#define FILE_K_AFTER_BA "updatedK.txt"
#define FILE_XYZ_AFTER_BA "updatedXYZ.txt"
#define FILE_BA_STRUCT_TEMP "updatedMOT.txt"
#define FILE_BA_XYZ_TEMP "updatedXYZ.txt"

//
#define FILE_IMAGE_PATH "imagePath.txt"
#define FILE_HISTORY "history.txt"
#define FILE_LOG "procLog.txt"

#define FILE_INTER_ST_XML "interStereoProj.xml"

#include <string>
using namespace std;

class CProjParam {
public:
    string m_strProjDir; // output project path
    string m_strSep;     // separate char which could be '/' or '\'
    string m_strImgL;    // image file path
    string m_strImgR;
    string m_strCalL;    // calibration data file path
    string m_strCalR;
    string m_strTPFile;  // tiepoint file path
    string m_strRsc;     // resource file path
    string m_strCurWD;   // current working dir
};


#endif // CPROJPARAM_H
