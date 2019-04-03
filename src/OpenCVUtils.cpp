#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "OpenCVUtils.h"

OpenCVUtils::OpenCVUtils(FrameDimension* pDepthFrameDimension, DepthBuffer* pDepthBuf, FrameDimension* pColorFrameDimension, ColorBuffer* pColorBuf) :
				_pDepthFrameDimension(NULL), _pDepthBuf(NULL), _pColorFrameDimension(NULL), _pColorBuf(pColorBuf), _pColorBGRBuf(NULL) {
	_pDepthFrameDimension 	= pDepthFrameDimension;
	_pDepthBuf 				= pDepthBuf;
	_pColorFrameDimension	= pColorFrameDimension;
	_pColorBuf				= pColorBuf;
	if (IsColorBufferAvailable()) {
		_pColorBGRBuf			= new ColorBuffer[(int)_pColorFrameDimension->Height * (int)_pColorFrameDimension->Width];
	}
}

OpenCVUtils::~OpenCVUtils() {
	delete _pColorBGRBuf;
}

ColorBuffer* OpenCVUtils::ConvertColorBufferFromRGBtoBGR(ColorBuffer* pColorBuf) {
	if (IsColorBufferAvailable()) {
		ColorBuffer* pColorBufTemp = NULL;
		if (NULL != pColorBuf) {
			pColorBufTemp = pColorBuf;
		} else {
			pColorBufTemp = _pColorBuf;
		}
		cv::Mat color_mat(_pColorFrameDimension->Height, _pColorFrameDimension->Width, CV_8UC3, pColorBufTemp);
		// Convert to BGR format for OpenCV
		cv::cvtColor(color_mat, color_mat, CV_RGB2BGR);
		for (unsigned v = 0; v < _pColorFrameDimension->Height; ++v) {
			for (unsigned u = 0; u < _pColorFrameDimension->Width; ++u)  {
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
	   	cv::Mat color_mat(_pColorFrameDimension->Height, _pColorFrameDimension->Width, CV_8UC3, _pColorBuf);
	    // Convert to BGR format for OpenCV
	    cv::cvtColor(color_mat, color_mat, CV_RGB2BGR);
	    if (NULL != FileNameWithPath) {
	    	cv::imwrite(FileNameWithPath, color_mat);
	    } else {
	    	cv::imshow("Color", color_mat);
		}
	}
}

void OpenCVUtils::StoreOrShowDepthBuffer(char* FileNameWithPath) {
	// Copy frame data to OpenCV mat
    cv::Mat depth_mat(_pDepthFrameDimension->Height, _pDepthFrameDimension->Width, CV_16U, _pDepthBuf);
    cv::cvtColor(depth_mat, depth_mat, CV_GRAY2BGR);
    if (NULL != FileNameWithPath) {
    	cv::imwrite(FileNameWithPath, depth_mat);
    } else {
    	cv::imshow("Color", depth_mat);
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
