#ifndef OPENCV_UTILS_H
#define OPENCV_UTILS_H

#include "Common.h"

class OpenCVUtils {
	public:
		~OpenCVUtils();
		OpenCVUtils(FrameDimension* pDepthFrameDimension, DepthBuffer* pDepthBuf, FrameDimension* pColorFrameDimension = NULL, ColorBuffer* pColorBuf = NULL);
		void 			ShowDepthBufferAsImage();
		void 			StoreDepthBufferAsImage(char* FileNameWithPath);
		void 			ShowColorBufferAsImage();
		void 			StoreColorBufferAsImage(char* FileNameWithPath);
		ColorBuffer* 	ConvertColorBufferFromRGBtoBGR(ColorBuffer* pColorBuf = NULL);

	private:
		FrameDimension* _pDepthFrameDimension;
		DepthBuffer* 	_pDepthBuf;
		FrameDimension* _pColorFrameDimension;
		ColorBuffer* 	_pColorBuf;
		ColorBuffer*	_pColorBGRBuf;
		bool 			IsColorBufferAvailable() {	return ((NULL != _pColorBuf && NULL !=  _pColorFrameDimension) ? true : false);	}
		void			StoreOrShowDepthBuffer(char* FileNameWithPath);
		void			StoreOrShowColorBuffer(char* FileNameWithPath);
};

#endif //OPENCV_UTILS_H