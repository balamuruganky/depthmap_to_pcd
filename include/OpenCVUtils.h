#ifndef OPENCV_UTILS_H
#define OPENCV_UTILS_H

#include "Common.h"

class OpenCVUtils {
	public:
		~OpenCVUtils();
		OpenCVUtils(DepthBufferInfo *pDepthBufInfo, ColorBufferInfo *pColorBufInfo);
		void 			ShowDepthBufferAsImage();
		void 			StoreDepthBufferAsImage(char* FileNameWithPath);
		void 			ShowColorBufferAsImage();
		void 			StoreColorBufferAsImage(char* FileNameWithPath);
		ColorBuffer* 	ConvertColorBufferFromRGBtoBGR(ColorBufferInfo* ColorBufferInfo = NULL);

	private:
		DepthBufferInfo* 	_pDepthBufInfo;
		ColorBufferInfo* 	_pColorBufInfo;
		ColorBuffer*		_pColorBGRBuf;
		bool 			IsColorBufferAvailable() {	return ((NULL != _pColorBufInfo) ? true : false);	}
		void			StoreOrShowDepthBuffer(char* FileNameWithPath);
		void			StoreOrShowColorBuffer(char* FileNameWithPath);
};

#endif //OPENCV_UTILS_H