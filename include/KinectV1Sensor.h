#ifndef KINECTV1_SENSOR_H
#define KINECTV1_SENSOR_H

#include <libfreenect.hpp>
#include <libfreenect.h>
#include <libfreenect_registration.h>

#include "Common.h"
#include "IDepthSensorBuilder.h"
#include "Thread.h"
#include "CriticalSection.h"
#include "ThreadSyncSemaphore.h"
#include "Condition.h"

typedef uint16_t freenect_depth;
typedef uint8_t freenect_packed_depth;
typedef uint8_t freenect_pixel;

#define FREENECT_FRAME_W 640
#define FREENECT_FRAME_H 480
#define FREENECT_FRAME_PIX (FREENECT_FRAME_H*FREENECT_FRAME_W)
#define FREENECT_RGB_SIZE (FREENECT_FRAME_PIX*3)
#define FREENECT_DEPTH_SIZE (FREENECT_FRAME_PIX*sizeof(freenect_depth))
#define FREENECT_PACKED_DEPTH_11_SIZE 422400

class KinectV1SensorHelper : public Freenect::FreenectDevice {
	public:
		KinectV1SensorHelper(freenect_context *_ctx, int _index);
		~KinectV1SensorHelper();

		freenect_pixel* 	_pRGB;
		freenect_depth* 	_pDepth;

		CriticalSection 	GetCriticalSection() 							{ return _oCS; 									}
		Condition  			GetCondition() 		 							{ return _oCondition; 							}
		bool 				IsColorFrameReceived() 							{ return _bColorFrameReceived; 					}
		bool 				IsDepthFrameReceived() 							{ return _bDepthFrameReceived; 					}
		void 				SetColorFrameReceived(bool bColorFrameReceived) { _bColorFrameReceived = bColorFrameReceived; 	}
		void 				SetDepthFrameReceived(bool bDepthFrameReceived) { _bDepthFrameReceived = bDepthFrameReceived; 	}

	private:
		void DepthCallback(freenect_depth* depth, uint32_t timestamp);
		void RGBCallback(freenect_pixel* rgb, uint32_t timestamp);

		CriticalSection		_oCS;
		Condition			_oCondition;
		bool				_bDepthFrameReceived;
		bool				_bColorFrameReceived;
};

class KinectV1Sensor: public IDepthSensorBuilder, public Thread {
	public:
		KinectV1Sensor();
		~KinectV1Sensor();

		int8_t  			InitDepthSensor();
		int8_t  			DeInitDepthSensor();
		DepthBuffer*  		GetDepthBuffer();
		FrameDimension*	  	GetDepthFrameDimension();
		ColorBuffer*  		GetColorBuffer();
		FrameDimension*	  	GetColorFrameDimension();
		CameraParameters* 	GetCameraParameters();
		int32_t				WaitForBufferStreams(uint16_t TimeOutInSeconds);

	private:
		void  					Run();
		int 					SetCameraParams();
		void 					SetDepthFrameDimension(int16_t width, int16_t height);
		void 					SetColorFrameDimension(int16_t width, int16_t height);
		KinectV1SensorHelper*	_pDevice;
		CameraParameters*		_pCameraParams;
		KinectV1Parameters*		_pKinectV1Params;
		FrameDimension			_oDepthFrameDimension;
		FrameDimension			_oColorFrameDimension;
		ThreadSyncSemaphore 	_oThreadSemaphore;

		freenect_frame_mode 	_oDepthFrameMode;
		freenect_frame_mode 	_oVideoFrameMode;
};

#endif // KINECTV1_SENSOR_H