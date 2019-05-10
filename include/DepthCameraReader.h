#ifndef DEPTH_CAMERA_READER_H
#define DEPTH_CAMERA_READER_H

#include "Common.h"
#include "IDepthSensorBuilder.h"
#include "Thread.h"

class DepthCameraReader : public Thread {
	public:
		DepthCameraReader(IDepthSensorBuilder* pDepthSensorBuilder);
		~DepthCameraReader();

	private:
		IDepthSensorBuilder* 	_pDepthSensorBuilder;

		DepthBufferInfo		_oDepthBufferInfo;
		ColorBufferInfo		_oColorBufferInfo;
		//FrameDimension*			_pDepthBufferDimension;
		//FrameDimension*			_pColorBufferDimension;

		void 					Run();
};

#endif // DEPTH_CAMERA_READER_H