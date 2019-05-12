#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "Common.h"
#include "CameraParameters.h"

class PCLUtils
{
public:
  PCLUtils (DepthBufferInfo * pDepthBufInfo, ColorBufferInfo * pColorBufInfo,
	    CameraParameters * pCameraParameters);
  void SavePCD ();

private:
    DepthBufferInfo * _pDepthBufInfo;
  ColorBufferInfo *_pColorBufInfo;
  FOVParameters *_pFOVParams;
  IntrinsicParameters *_pIntrinsicParameters;
  KinectV1Parameters *_pKinectV1Parameters;
  CameraParameters *_pCameraParameters;
  ColorBuffer _RGBValues;
  CameraParametersType _CameraParamType;
  void UpdateColorToPCD (pcl::PointXYZRGB & pt, int depth_idx);
  void SetPointCloudRGBValue (ColorBuffer oColorBuf);
    pcl::PointCloud <
    pcl::PointXYZRGB >::Ptr CreateEmptyPointCloud (int DepthFrameHeight,
						   int DepthFrameWidth);
  void GeneratePCDFileUsingFoVParams ();
  void GeneratePCDFileUsingIntrinsicParams ();
  void GeneratePCDFileUsingKinectV1Parameters ();

};

#endif //PCL_UTILS_H
