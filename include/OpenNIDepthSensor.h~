#ifndef OPENNI_DEPTH_SENSOR_H
#define OPENNI_DEPTH_SENSOR_H

#include <OpenNI.h>
#include "Common.h"
#include "IDepthSensorBuilder.h"
#include "Thread.h"
#include "CameraParameters.h"
#include "ThreadSyncSemaphore.h"

class OpenNIDepthSensor : public IDepthSensorBuilder, public Thread {
	public:
		OpenNIDepthSensor();
		~OpenNIDepthSensor();

		int8_t  			InitDepthSensor();
		int8_t  			DeInitDepthSensor();
		DepthBuffer*  		GetDepthBuffer();
		FrameDimension*	  	GetDepthFrameDimension();
		ColorBuffer*  		GetColorBuffer();
		FrameDimension*	  	GetColorFrameDimension();
		CameraParameters* 	GetCameraParameters();
		int32_t				WaitForBufferStreams(uint16_t TimeOutInSeconds);

	private:
		int8_t  InitOpenNI();
		int8_t  InitDevice();
		int8_t  InitDepthStream();
		int8_t  InitColorStream();
		void  	Run();
		void    DeInitOpenNI();
		void    DeInitDevice();
		void    CapturePsenseDepthFrame();
		void    CapturePsenseColorFrame();
		void	SetCameraParams();
		void	SetDepthFrameDimension();
		void	SetColorFrameDimension();

		openni::Device*        	device_;
		openni::VideoStream*   	depth_stream_;
		openni::VideoStream*   	color_stream_;
		openni::VideoFrameRef* 	depth_frame_;
		openni::VideoFrameRef* 	color_frame_;
		openni::RGB888Pixel*   	dev_rgbbuf_ptr_;
		openni::DepthPixel*    	dev_depthbuf_ptr_;

		CameraParameters*		_pCameraParams;
		FOVParameters*			_pFovParams;
		FrameDimension			_oDepthFrameDimension;
		FrameDimension			_oColorFrameDimension;
		ThreadSyncSemaphore 	_oThreadSemaphore;
};

#endif