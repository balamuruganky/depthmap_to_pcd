#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "Common.h"

class PCLUtils {
	public:
		PCLUtils(FrameDimension* pDepthFrameDimension, FOVParameters* pFOVParams, DepthBuffer* pDepthBuf, ColorBuffer* pColorBuf = NULL);
		PCLUtils(FrameDimension* pDepthFrameDimension, FOVParameters* pFOVParams, DepthBuffer* pDepthBuf, ColorBuffer RGBValues);
		PCLUtils(FrameDimension* pDepthFrameDimension, IntrinsicParameters* pIntrinsicParameters, DepthBuffer* pDepthBuf, ColorBuffer* pColorBuf = NULL);
		PCLUtils(FrameDimension* pDepthFrameDimension, IntrinsicParameters* pIntrinsicParameters, DepthBuffer* pDepthBuf, ColorBuffer RGBValues);
		void GeneratePCDFileUsingFoVParams();
		void GeneratePCDFileUsingIntrinsicParams();

	private:
		FrameDimension* 		_pDepthFrameDimension;
		FOVParameters* 			_pFOVParams;
		IntrinsicParameters*	_pIntrinsicParameters;
		DepthBuffer* 			_pDepthBuf;
		ColorBuffer* 			_pColorBuf;
		ColorBuffer				_RGBValues;
		void UpdateColorToPCD(pcl::PointXYZRGB& pt, int depth_idx);
};

#endif //PCL_UTILS_H