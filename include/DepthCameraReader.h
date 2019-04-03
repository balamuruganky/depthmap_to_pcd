#ifndef DEPTH_CAMERA_READER_H
#define DEPTH_CAMERA_READER_H

#include "IDepthSensorBuilder.h"
#include "Thread.h"

class DepthCameraReader : public Thread {
	public:
		DepthCameraReader(IDepthSensorBuilder* pDepthSensorBuilder);
		~DepthCameraReader();

	private:
		IDepthSensorBuilder* 	_pDepthSensorBuilder;

		DepthBuffer*			_pDepthBuffer;
		ColorBuffer*			_pColorBuffer;
		FrameDimension*			_pDepthBufferDimension;
		FrameDimension*			_pColorBufferDimension;

		void 					Run();
};

#endif // DEPTH_CAMERA_READER_H