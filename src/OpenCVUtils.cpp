#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "OpenCVUtils.h"

OpenCVUtils::OpenCVUtils(DepthBufferInfo *pDepthBufInfo, ColorBufferInfo *pColorBufInfo) :
				_pDepthBufInfo(NULL), _pColorBufInfo(NULL), _pColorBGRBuf(NULL) {
	_pColorBufInfo 	= pColorBufInfo;
	_pDepthBufInfo	= pDepthBufInfo;
	if (IsColorBufferAvailable()) {
		_pColorBGRBuf	= new ColorBuffer[(int)_pColorBufInfo->pFrameDim->Height * (int)_pColorBufInfo->pFrameDim->Width];
	}
}

OpenCVUtils::~OpenCVUtils() {
	delete _pColorBGRBuf;
}

ColorBuffer* OpenCVUtils::ConvertColorBufferFromRGBtoBGR(ColorBufferInfo* pColorBufInfo) {
	if (IsColorBufferAvailable()) {
		ColorBufferInfo* pColorBufTemp = NULL;
		if (NULL != pColorBufInfo) {
			pColorBufTemp = pColorBufInfo;
		} else {
			pColorBufTemp = _pColorBufInfo;
		}
		cv::Mat color_mat(_pColorBufInfo->pFrameDim->Height, _pColorBufInfo->pFrameDim->Width, CV_8UC3, pColorBufTemp->pColorBuf);
		// Convert to BGR format for OpenCV
		cv::cvtColor(color_mat, color_mat, CV_RGB2BGR);
		for (unsigned v = 0; v < _pColorBufInfo->pFrameDim->Height; ++v) {
			for (unsigned u = 0; u < _pColorBufInfo->pFrameDim->Width; ++u)  {
				_pColorBGRBuf->Red 		= color_mat.at<cv::Vec3b>(v,u)[0];
				_pColorBGRBuf->Green 	= color_mat.at<cv::Vec3b>(v,u)[1];
				_pColorBGRBuf->Blue 	= color_mat.at<cv::Vec3b>(v,u)[2];
			}
		}
	} else {
		printf ("No colour buffer available!!!");
	}

	return _pColorBGRBuf;
}

void OpenCVUtils::StoreOrShowColorBuffer(char* FileNameWithPath) {
	if (IsColorBufferAvailable()) {
		// Make mat from camera data
	   	cv::Mat color_mat(_pColorBufInfo->pFrameDim->Height, _pColorBufInfo->pFrameDim->Width, CV_8UC3, _pColorBufInfo->pColorBuf);
	    // Convert to BGR format for OpenCV
	    cv::cvtColor(color_mat, color_mat, CV_RGB2BGR);
		ShowOrStoreMat(color_mat, FileNameWithPath);
	}
}

void OpenCVUtils::StoreOrShowDepthBuffer(char* FileNameWithPath) {
	// Copy frame data to OpenCV mat
    cv::Mat depth_mat(_pDepthBufInfo->pFrameDim->Height, _pDepthBufInfo->pFrameDim->Width, CV_16U, _pDepthBufInfo->pDepthBuf);
    cv::cvtColor(depth_mat, depth_mat, CV_GRAY2BGR);
	ShowOrStoreMat(depth_mat, FileNameWithPath);
}

void OpenCVUtils::ShowOrStoreMat(cv::Mat mat, char* FileNameWithPath) {
	if (NULL != FileNameWithPath) {
		cv::imwrite(FileNameWithPath, mat);
	} else {
		cv::imshow(FileNameWithPath, mat);
	}
}

void OpenCVUtils::ShowDepthBufferAsImage() {
	StoreOrShowDepthBuffer(NULL);
}

void OpenCVUtils::StoreDepthBufferAsImage(char* FileNameWithPath) {
	StoreOrShowDepthBuffer(FileNameWithPath);
}

void OpenCVUtils::ShowColorBufferAsImage() {
	StoreOrShowColorBuffer(NULL);
}

void OpenCVUtils::StoreColorBufferAsImage(char* FileNameWithPath) {
	StoreOrShowColorBuffer(FileNameWithPath);
}
