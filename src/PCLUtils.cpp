#include "Utils.h"
#include "PCLUtils.h"

#define MM_TO_METERS 0.001f

PCLUtils::PCLUtils(FrameDimension* pDepthFrameDimension, FOVParameters* pFOVParams, DepthBuffer* pDepthBuf, ColorBuffer* pColorBuf) : 
		_pDepthFrameDimension(NULL), _pFOVParams(NULL), _pDepthBuf(NULL), _pColorBuf(NULL), _RGBValues({0,0,0}) {
	_pDepthFrameDimension = pDepthFrameDimension;
	_pFOVParams = pFOVParams;
	_pDepthBuf = pDepthBuf;
	_pColorBuf = pColorBuf;
}

PCLUtils::PCLUtils(FrameDimension* pDepthFrameDimension, FOVParameters* pFOVParams, DepthBuffer* pDepthBuf, ColorBuffer RGBValues) :
		_pDepthFrameDimension(NULL), _pFOVParams(NULL), _pDepthBuf(NULL), _pColorBuf(NULL), _RGBValues({0,0,0}) {
	_pDepthFrameDimension = pDepthFrameDimension;
	_pFOVParams = pFOVParams;
	_pDepthBuf = pDepthBuf;
	_RGBValues = RGBValues;
}

PCLUtils::PCLUtils(FrameDimension* pDepthFrameDimension, IntrinsicParameters* pIntrinsicParameters, DepthBuffer* pDepthBuf, ColorBuffer* pColorBuf) : 
		_pDepthFrameDimension(NULL), _pIntrinsicParameters(NULL), _pDepthBuf(NULL), _pColorBuf(NULL), _RGBValues({0,0,0}) {
	_pDepthFrameDimension = pDepthFrameDimension;
	_pIntrinsicParameters = pIntrinsicParameters;
	_pDepthBuf = pDepthBuf;
	_pColorBuf = pColorBuf;
}

PCLUtils::PCLUtils(FrameDimension* pDepthFrameDimension, IntrinsicParameters* pIntrinsicParameters, DepthBuffer* pDepthBuf, ColorBuffer RGBValues) :
		_pDepthFrameDimension(NULL), _pIntrinsicParameters(NULL), _pDepthBuf(NULL), _pColorBuf(NULL), _RGBValues({0,0,0}) {
	_pDepthFrameDimension = pDepthFrameDimension;
	_pIntrinsicParameters = pIntrinsicParameters;
	_pDepthBuf = pDepthBuf;
	_RGBValues = RGBValues;
}

PCLUtils::PCLUtils(FrameDimension* pDepthFrameDimension, KinectV1Parameters* pKinectV1Parameters, DepthBuffer* pDepthBuf, ColorBuffer* pColorBuf) : 
		_pDepthFrameDimension(NULL), _pKinectV1Parameters(NULL), _pDepthBuf(NULL), _pColorBuf(NULL), _RGBValues({0,0,0}) {
	_pDepthFrameDimension = pDepthFrameDimension;
	_pKinectV1Parameters = pKinectV1Parameters;
	_pDepthBuf = pDepthBuf;
	_pColorBuf = pColorBuf;
}

PCLUtils::PCLUtils(FrameDimension* pDepthFrameDimension, KinectV1Parameters* pKinectV1Parameters, DepthBuffer* pDepthBuf, ColorBuffer RGBValues) :
		_pDepthFrameDimension(NULL), _pKinectV1Parameters(NULL), _pDepthBuf(NULL), _pColorBuf(NULL), _RGBValues({0,0,0}) {
	_pDepthFrameDimension = pDepthFrameDimension;
	_pKinectV1Parameters = pKinectV1Parameters;
	_pDepthBuf = pDepthBuf;
	_RGBValues = RGBValues;
}

void PCLUtils::GeneratePCDFileUsingKinectV1Parameters() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud (new pcl::PointCloud <pcl::PointXYZRGB>);
	int DepthFrameWidth = _pDepthFrameDimension->Width;
	int DepthFrameHeight = _pDepthFrameDimension->Height;
	float RefPixSize = _pKinectV1Parameters->RefPixelSize;
	float RefDistance = _pKinectV1Parameters->RefDistance;
	pointcloud->width = DepthFrameWidth;
	pointcloud->height = DepthFrameHeight;
	pointcloud->points.resize (DepthFrameHeight * DepthFrameWidth);

	unsigned depth_idx = 0;
	for (unsigned v = 0; v < DepthFrameHeight; ++v) {
		for (unsigned u = 0; u < DepthFrameWidth; ++u, ++depth_idx)  {
			pcl::PointXYZRGB& pt = pointcloud->points[depth_idx];
			pt.z = _pDepthBuf[depth_idx] * MM_TO_METERS;
		    if (pt.z != 0) {
				double factor = 2 * RefPixSize * pt.z / RefDistance;
				pt.x = (double)(u - DepthFrameWidth / 2) * factor;
				pt.y = (double)(v - DepthFrameHeight / 2) * factor;
			} else {
		    	pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
		    }

			UpdateColorToPCD(pt, depth_idx);
		}
	}

 	pcl::io::savePCDFile (Utils::PrepareUniqueFileName("pcd", "pcd"), *pointcloud);
}

void PCLUtils::GeneratePCDFileUsingFoVParams() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud (new pcl::PointCloud <pcl::PointXYZRGB>);
	int DepthFrameWidth = _pDepthFrameDimension->Width;
	int DepthFrameHeight = _pDepthFrameDimension->Height;
	float DepthHFoV = _pFOVParams->DepthHorizontalFOV;
	float DepthVFoV = _pFOVParams->DepthVerticalFOV;
	pointcloud->width = DepthFrameWidth;
	pointcloud->height = DepthFrameHeight;
	pointcloud->points.resize (DepthFrameHeight * DepthFrameWidth);
  
	unsigned depth_idx = 0;
	for (unsigned v = 0; v < DepthFrameHeight; ++v) {
		for (unsigned u = 0; u < DepthFrameWidth; ++u, ++depth_idx)  {
			pcl::PointXYZRGB& pt = pointcloud->points[depth_idx];
			pt.z = _pDepthBuf[depth_idx] * MM_TO_METERS;
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud (new pcl::PointCloud <pcl::PointXYZRGB>);
	int DepthFrameWidth = _pDepthFrameDimension->Width;
	int DepthFrameHeight = _pDepthFrameDimension->Height;
	float Fx = _pIntrinsicParameters->Fx;
	float Fy = _pIntrinsicParameters->Fy;
	float Cx = _pIntrinsicParameters->Cx;
	float Cy = _pIntrinsicParameters->Cy;

	pointcloud->width = DepthFrameWidth;
	pointcloud->height = DepthFrameHeight;
	pointcloud->points.resize (DepthFrameHeight * DepthFrameWidth);
  
	unsigned depth_idx = 0;
	for (unsigned v = 0; v < DepthFrameHeight; ++v) {
		for (unsigned u = 0; u < DepthFrameWidth; ++u, ++depth_idx)  {
		    pcl::PointXYZRGB& pt = pointcloud->points[depth_idx];
		    pt.z = _pDepthBuf[depth_idx] * MM_TO_METERS;
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

void PCLUtils::UpdateColorToPCD(pcl::PointXYZRGB& pt, int depth_idx) {
	//
	// Assign colour values
	//
	if (_pColorBuf != NULL) {
		pt.r = _pColorBuf[depth_idx].Red;
		pt.g = _pColorBuf[depth_idx].Green;
		pt.b = _pColorBuf[depth_idx].Blue;
		//printf ("%d %d %d\n", pt.r, pt.g, pt.b);
	} else {
		if (_RGBValues.Red != 0 && _RGBValues.Green != 0 && _RGBValues.Blue != 0) {
			pt.r = _RGBValues.Red;
			pt.g = _RGBValues.Green;
			pt.b = _RGBValues.Blue;
		} else {
			pt.r = pt.g = pt.b = 255;
		}
	}
}

