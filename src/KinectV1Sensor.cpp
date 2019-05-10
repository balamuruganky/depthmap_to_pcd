#include "KinectV1Sensor.h"

KinectV1SensorHelper::KinectV1SensorHelper(freenect_context *_ctx, int _index)
	 		: Freenect::FreenectDevice(_ctx, _index), _bDepthFrameReceived(false), _bColorFrameReceived(false) {
	 _oCondition.SetMutex(_oCS.GetObject());
	 _oRGB.clear();
	 _oDepth.clear();
}

KinectV1SensorHelper::~KinectV1SensorHelper() {

}

void KinectV1SensorHelper::DepthCallback(void* _depth, uint32_t timestamp) {
	_oCS.Lock();
	_oDepth.clear();
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	for (size_t i = 0; i < (getDepthBufferSize() / 2); i++) {
		_oDepth.push_back(depth[i]);
	}
	_bDepthFrameReceived = true;
	_oCondition.ConditionalSignal();
	_oCS.Unlock();
}

void KinectV1SensorHelper::VideoCallback(void* _rgb, uint32_t timestamp) {
	_oCS.Lock();
	_oRGB.clear();
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);
	//std::copy(rgb, rgb+getVideoBufferSize(), _oRGB.begin());
	for (size_t i = 0; i < getVideoBufferSize(); i++) {
		_oRGB.push_back(rgb[i]);
	}
	_bColorFrameReceived = true;
	_oCondition.ConditionalSignal();
	_oCS.Unlock();
}

KinectV1Sensor::KinectV1Sensor() : _pCameraParams(NULL), _pKinectV1Params(NULL), _pDeviceHandle(NULL), _pDevice(NULL), _pColorBuffer(NULL) {
	_pCameraParams = new CameraParameters();
	_pKinectV1Params = new KinectV1Parameters();

	_oDepthFrameDimension = _oColorFrameDimension = {0,0};

	//SetCameraParams();

	_pDeviceHandle = &_oFreenect.createDevice<KinectV1SensorHelper>(0);
	_pDevice = const_cast<freenect_device*>(_pDeviceHandle->getDevice());

	SetCameraParameters();
	SetDepthFrameDimension();
	SetColorFrameDimension();

	_pColorBuffer = new ColorBuffer[(int)_oColorFrameDimension.Width * (int)_oColorFrameDimension.Height];
}

KinectV1Sensor::~KinectV1Sensor() {
	delete[] 	_pColorBuffer;
	delete 		_pKinectV1Params;
	delete 		_pCameraParams;
}

int8_t KinectV1Sensor::InitDepthSensor() {
	_pDeviceHandle->startVideo();
	_pDeviceHandle->startDepth();
	StartThread();
}

int8_t KinectV1Sensor::DeInitDepthSensor() {
	_pDeviceHandle->stopVideo();
	_pDeviceHandle->stopDepth();
}

FrameDimension* KinectV1Sensor::GetDepthFrameDimension() {
	return (&_oDepthFrameDimension);
}

void KinectV1Sensor::SetDepthFrameDimension() {
	freenect_frame_mode oDepthFrameMode = freenect_get_current_depth_mode(_pDevice);
	oDepthFrameMode.depth_format = FREENECT_DEPTH_REGISTERED;
	oDepthFrameMode.resolution 	 = FREENECT_RESOLUTION_MEDIUM;
	freenect_set_depth_mode(_pDevice, oDepthFrameMode);

	oDepthFrameMode = freenect_get_current_depth_mode (_pDevice);

	_oDepthFrameDimension.Width 	= (int)oDepthFrameMode.width;
	_oDepthFrameDimension.Height 	= (int)oDepthFrameMode.height;
}

FrameDimension* KinectV1Sensor::GetColorFrameDimension() {
	return (&_oColorFrameDimension);
}

void KinectV1Sensor::SetColorFrameDimension() {
	freenect_frame_mode oVideoFrameMode = freenect_get_current_video_mode(_pDevice);
	oVideoFrameMode.video_format = FREENECT_VIDEO_RGB;
	oVideoFrameMode.resolution 	 = FREENECT_RESOLUTION_MEDIUM;
	freenect_set_video_mode(_pDevice, oVideoFrameMode);

	oVideoFrameMode = freenect_get_current_video_mode (_pDevice);

	_oColorFrameDimension.Width 	= (int)oVideoFrameMode.width;
	_oColorFrameDimension.Height 	= (int)oVideoFrameMode.height;
}

CameraParameters* KinectV1Sensor::GetCameraParameters() {
	return _pCameraParams;
}

void KinectV1Sensor::SetCameraParameters() {
	freenect_registration regst = freenect_copy_registration(_pDevice);
	_pKinectV1Params->RefPixelSize = regst.zero_plane_info.reference_pixel_size;
	_pKinectV1Params->RefDistance  = regst.zero_plane_info.reference_distance;
    _pCameraParams->SetKinectV1ParametersInstance(_pKinectV1Params);
}

DepthBuffer* KinectV1Sensor::GetDepthBuffer() {
	return (DepthBuffer*)_pDeviceHandle->_oDepth.data();
}

ColorBuffer* KinectV1Sensor::GetColorBuffer() {
	size_t i = 0;
	uint8_t* pRGB = _pDeviceHandle->_oRGB.data();
	for (size_t v = 0 ; v < _oColorFrameDimension.Height; v++) {
		for ( size_t u = 0 ; u < _oColorFrameDimension.Width ; u++, i++) {
			_pColorBuffer[i].Red 	= pRGB[(i*3)];
			_pColorBuffer[i].Green 	= pRGB[(i*3)+1];
			_pColorBuffer[i].Blue 	= pRGB[(i*3)+2];
		}
	}
	return _pColorBuffer;
}

int32_t	KinectV1Sensor::WaitForBufferStreams(uint16_t TimeOutInSeconds) {
	return _oThreadSemaphore.SemaphoreTimedWait(TimeOutInSeconds);
}

void KinectV1Sensor::Run() {
	if (_pDeviceHandle != NULL) {
		bool isDepthAvailable = false, isRGBAvailable = false;
		_pDeviceHandle->GetCriticalSection().Lock();
		while (!_pDeviceHandle->IsColorFrameReceived() && !_pDeviceHandle->IsDepthFrameReceived())
		    _pDeviceHandle->GetCondition().ConditionalWait();

		if (_pDeviceHandle->IsColorFrameReceived()) {
		    _pDeviceHandle->SetColorFrameReceived(false);
		    isRGBAvailable = true;
		}

		if (_pDeviceHandle->IsDepthFrameReceived()) {
		    _pDeviceHandle->SetDepthFrameReceived(false);
		    isDepthAvailable = true;
		}
		if (isDepthAvailable && isRGBAvailable) {
			_oThreadSemaphore.SemaphorePost();
			isDepthAvailable = isRGBAvailable = false;
		}
		
		_pDeviceHandle->GetCriticalSection().Unlock();
	}
}