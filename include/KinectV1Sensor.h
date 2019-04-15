#ifndef KINECTV1_SENSOR_H
#define KINECTV1_SENSOR_H

#include <libfreenect.hpp>
#include <libfreenect.h>
#include <libfreenect_registration.h>
#include <vector>
using namespace std;

#include "Common.h"
#include "IDepthSensorBuilder.h"
#include "Thread.h"
#include "CriticalSection.h"
#include "ThreadSyncSemaphore.h"
#include "Condition.h"

class KinectV1SensorHelper : public Freenect::FreenectDevice {
	public:
		KinectV1SensorHelper(freenect_context *_ctx, int _index);
		~KinectV1SensorHelper();

		std::vector<uint8_t> 	_oRGB;
		std::vector<uint16_t> 	_oDepth;

		CriticalSection 	GetCriticalSection() 							{ return _oCS; 									}
		Condition  			GetCondition() 		 							{ return _oCondition; 							}
		bool 				IsColorFrameReceived() 							{ return _bColorFrameReceived; 					}
		bool 				IsDepthFrameReceived() 							{ return _bDepthFrameReceived; 					}
		void 				SetColorFrameReceived(bool bColorFrameReceived) { _bColorFrameReceived = bColorFrameReceived; 	}
		void 				SetDepthFrameReceived(bool bDepthFrameReceived) { _bDepthFrameReceived = bDepthFrameReceived; 	}

	private:
		void VideoCallback(void* _rgb, uint32_t timestamp);
		void DepthCallback(void* _depth, uint32_t timestamp);

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
		void					SetCameraParameters();
		void 					SetDepthFrameDimension();
		void 					SetColorFrameDimension();
		KinectV1SensorHelper*	_pDeviceHandle;
		freenect_device*		_pDevice;
		Freenect::Freenect 		_oFreenect;
		CameraParameters*		_pCameraParams;
		KinectV1Parameters*		_pKinectV1Params;
		FrameDimension			_oDepthFrameDimension;
		FrameDimension			_oColorFrameDimension;
		ThreadSyncSemaphore 	_oThreadSemaphore;
		ColorBuffer*			_pColorBuffer;

		freenect_frame_mode 	_oDepthFrameMode;
		freenect_frame_mode 	_oVideoFrameMode;
};

#endif // KINECTV1_SENSOR_H