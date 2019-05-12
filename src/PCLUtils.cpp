#include "Utils.h"
#include "PCLUtils.h"

#define MM_TO_METERS 0.001f

PCLUtils::PCLUtils(DepthBufferInfo *pDepthBufInfo, ColorBufferInfo *pColorBufInfo, CameraParameters* pCameraParameters) : 
		_pCameraParameters(NULL), _pDepthBufInfo(NULL), _pColorBufInfo(NULL), _RGBValues({0,255,0}) {
	if (NULL != pCameraParameters) {
		_CameraParamType = pCameraParameters->GetCameraParameterType();
		switch (_CameraParamType) {
			case CAMERA_FOV_PARAMETERS:
				_pFOVParams = pCameraParameters->GetFovParametersInstance();
				break;
			case CAMERA_INTRINSIC_PARAMETERS:
				_pIntrinsicParameters = pCameraParameters->GetIntrinsicParametersInstance();
				break;
			case CAMERA_KINECTV1_PARAMETERS:
				_pKinectV1Parameters = pCameraParameters->GetKinectV1ParametersInstance();
				break;
			default:
				printf ("Camera parameters not available!!!\n");
		}
	}
	_pDepthBufInfo = pDepthBufInfo;
	_pColorBufInfo = pColorBufInfo;
}

void PCLUtils::SetPointCloudRGBValue(ColorBuffer oColorBuf) {
	_RGBValues.Red 	= oColorBuf.Red;
	_RGBValues.Green = oColorBuf.Green;
	_RGBValues.Blue = oColorBuf.Blue;
}

void PCLUtils::SavePCD() {
	switch (_CameraParamType) {
		case CAMERA_FOV_PARAMETERS:
			GeneratePCDFileUsingFoVParams();
			break;
		case CAMERA_INTRINSIC_PARAMETERS:
			GeneratePCDFileUsingIntrinsicParams();
			break;
		case CAMERA_KINECTV1_PARAMETERS:
			GeneratePCDFileUsingKinectV1Parameters();
			break;
		default:
			printf ("Camera parameters not available!!!\n");
	}
}

void PCLUtils::GeneratePCDFileUsingKinectV1Parameters() {
	int DepthFrameWidth = (int)_pDepthBufInfo->pFrameDim->Width;
	int DepthFrameHeight = (int)_pDepthBufInfo->pFrameDim->Height;
	double RefPixSize = (double)_pKinectV1Parameters->GetRefPixelSize();
	double RefDistance = (double)_pKinectV1Parameters->GetRefDistance();
	double currDepth = 0.0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = CreateEmptyPointCloud(DepthFrameHeight, DepthFrameWidth);
	
	unsigned depth_idx = 0;
	for (int v = 0; v < DepthFrameHeight; ++v) {
		for (int u = 0; u < DepthFrameWidth; ++u, ++depth_idx)  {
			pcl::PointXYZRGB& pt = pointcloud->points[depth_idx];
			currDepth = _pDepthBufInfo->pDepthBuf[depth_idx];
		   if (currDepth != 0) {
				double factor = 2 * RefPixSize * currDepth / RefDistance;
				pt.x = (double)((u - (DepthFrameWidth / 2)) * factor);
				pt.y = (double)((v - (DepthFrameHeight /2)) * factor);
				pt.z = (currDepth);
			} else {
		    	pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
		   }

			UpdateColorToPCD(pt, depth_idx);
		}
	}

 	pcl::io::savePCDFile (Utils::PrepareUniqueFileName("pcd", "pcd"), *pointcloud);
}

void PCLUtils::GeneratePCDFileUsingFoVParams() {
	int DepthFrameWidth = _pDepthBufInfo->pFrameDim->Width;
	int DepthFrameHeight = _pDepthBufInfo->pFrameDim->Height;
	float DepthHFoV = _pFOVParams->GetDepthHorizontalFOV();
	float DepthVFoV = _pFOVParams->GetDepthVerticalFOV();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = CreateEmptyPointCloud(DepthFrameHeight, DepthFrameWidth);
  
	unsigned depth_idx = 0;
	for (unsigned v = 0; v < DepthFrameHeight; ++v) {
		for (unsigned u = 0; u < DepthFrameWidth; ++u, ++depth_idx)  {
			pcl::PointXYZRGB& pt = pointcloud->points[depth_idx];
			pt.z = _pDepthBufInfo->pDepthBuf[depth_idx] * MM_TO_METERS;
		    if (pt.z != 0) {
				int r_i = depth_idx / (int)DepthFrameWidth;
				int c_i = depth_idx % (int)DepthFrameWidth;

				//
				// Calculate X coordinate value
				//
				float alpha_h = (M_PI - DepthHFoV) / 2;
				float gamma_i_h = alpha_h + (float)c_i*(DepthHFoV / DepthFrameWidth);
				pt.x = pt.z / tan(gamma_i_h);

				//
				// Calculate Y coordinate value
				//
				float alpha_v = 2 * M_PI - (DepthVFoV / 2);
				float gamma_i_v = alpha_v + (float)r_i*(DepthVFoV / DepthFrameWidth);
				pt.y = pt.z * tan(gamma_i_v)*-1;
			} else {
		    	pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
		    }

			UpdateColorToPCD(pt, depth_idx);
		}
	}

 	pcl::io::savePCDFile (Utils::PrepareUniqueFileName("pcd", "pcd"), *pointcloud);
}

void PCLUtils::GeneratePCDFileUsingIntrinsicParams() {
	int DepthFrameWidth = _pDepthBufInfo->pFrameDim->Width;
	int DepthFrameHeight = _pDepthBufInfo->pFrameDim->Height;
	float Fx = _pIntrinsicParameters->GetFx();
	float Fy = _pIntrinsicParameters->GetFy();
	float Cx = _pIntrinsicParameters->GetCx();
	float Cy = _pIntrinsicParameters->GetCy();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = CreateEmptyPointCloud(DepthFrameHeight, DepthFrameWidth);
  
	unsigned depth_idx = 0;
	for (unsigned v = 0; v < DepthFrameHeight; ++v) {
		for (unsigned u = 0; u < DepthFrameWidth; ++u, ++depth_idx)  {
		    pcl::PointXYZRGB& pt = pointcloud->points[depth_idx];
		    pt.z = _pDepthBufInfo->pDepthBuf[depth_idx] * MM_TO_METERS;
		    if (pt.z != 0) {	             
				pt.x = pt.z * ((u - Cx) * Fx);
				pt.y = pt.z * ((v - Cy) * Fy);
		    } else {
		    	pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
		    }

			UpdateColorToPCD(pt, depth_idx);
		}
	}

 	pcl::io::savePCDFile (Utils::PrepareUniqueFileName("pcd", "pcd"), *pointcloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLUtils::CreateEmptyPointCloud (int DepthFrameHeight, int DepthFrameWidth) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud (new pcl::PointCloud <pcl::PointXYZRGB>);
	pointcloud->width = DepthFrameWidth;
	pointcloud->height = DepthFrameHeight;
	pointcloud->points.resize (DepthFrameHeight * DepthFrameWidth);
	return pointcloud;
}

void PCLUtils::UpdateColorToPCD(pcl::PointXYZRGB& pt, int depth_idx) {
	//
	// Assign colour values
	//
	if (_pColorBufInfo->pColorBuf != NULL) {
		pt.r = _pColorBufInfo->pColorBuf[depth_idx].Red;
		pt.g = _pColorBufInfo->pColorBuf[depth_idx].Green;
		pt.b = _pColorBufInfo->pColorBuf[depth_idx].Blue;
	} else {
		pt.r = _RGBValues.Red;
		pt.g = _RGBValues.Green;
		pt.b = _RGBValues.Blue;
	}
}

