#include "KinectV1Sensor.h"

KinectV1SensorHelper::KinectV1SensorHelper(freenect_context *_ctx, int _index)
	 		: Freenect::FreenectDevice(_ctx, _index), _pRGB(NULL), _pDepth(NULL) {
	 _oCondition.SetMutex(_oCS.GetObject());
}

KinectV1SensorHelper::~KinectV1SensorHelper() {

}

void KinectV1SensorHelper::DepthCallback (freenect_depth* depth, uint32_t timestamp) {
	_oCS.Lock();
	_pDepth = depth;
	_bDepthFrameReceived = true;
	_oCondition.ConditionalSignal();
	_oCS.Unlock();
}

void KinectV1SensorHelper::RGBCallback (freenect_pixel* rgb, uint32_t timestamp) {
	_oCS.Lock();
	_pRGB = rgb;
	_bColorFrameReceived = true;
	_oCondition.ConditionalSignal();
	_oCS.Unlock();
}

KinectV1Sensor::KinectV1Sensor() : _pCameraParams(NULL), _pKinectV1Params(NULL), _pDevice(NULL) {
	_pCameraParams = new CameraParameters();
	_pKinectV1Params = new KinectV1Parameters();
	
	_oDepthFrameDimension = _oColorFrameDimension = {0,0};

	SetCameraParams();

	Freenect::Freenect freenect;
	_pDevice = &freenect.createDevice<KinectV1SensorHelper>(0);
}

KinectV1Sensor::~KinectV1Sensor() {
	delete _pKinectV1Params;
	delete _pCameraParams;
}

int8_t KinectV1Sensor::InitDepthSensor() {
	_pDevice->startVideo();
	_pDevice->startDepth();

	StartThread();
}

int8_t KinectV1Sensor::DeInitDepthSensor() {
	_pDevice->stopVideo();
	_pDevice->stopDepth();
}

FrameDimension* KinectV1Sensor::GetDepthFrameDimension() {
	return (&_oDepthFrameDimension);
}

void KinectV1Sensor::SetDepthFrameDimension(int16_t width, int16_t height) {
	_oDepthFrameDimension.Width 	= (float)width;
	_oDepthFrameDimension.Height 	= (float)height;
}

FrameDimension* KinectV1Sensor::GetColorFrameDimension() {
	return (&_oColorFrameDimension);
}

void KinectV1Sensor::SetColorFrameDimension(int16_t width, int16_t height) {
	_oColorFrameDimension.Width 	= (float)width;
	_oColorFrameDimension.Height 	= (float)height;
}

CameraParameters* KinectV1Sensor::GetCameraParameters() {
	return _pCameraParams;
}

DepthBuffer* KinectV1Sensor::GetDepthBuffer() {
	return (DepthBuffer*)_pDevice->_pDepth;
}

ColorBuffer* KinectV1Sensor::GetColorBuffer() {
	return ((ColorBuffer*)_pDevice->_pRGB);
}

int32_t	KinectV1Sensor::WaitForBufferStreams(uint16_t TimeOutInSeconds) {
	return _oThreadSemaphore.SemaphoreTimedWait(TimeOutInSeconds);
}

int KinectV1Sensor::SetCameraParams() {
		// Initialize libfreenect.
	freenect_context* fn_ctx;
	int ret = freenect_init(&fn_ctx, NULL);
	if (ret < 0)
		return ret;

	// Show debug messages and use camera only.
	freenect_set_log_level(fn_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(fn_ctx, FREENECT_DEVICE_CAMERA);

	// Find out how many devices are connected.
	int num_devices = ret = freenect_num_devices(fn_ctx);
	if (ret < 0)
		return ret;
	if (num_devices == 0) {
		printf("No device found!\n");
		freenect_shutdown(fn_ctx);
		return 1;
	}

	// Open the first device.
	freenect_device* fn_dev;
	ret = freenect_open_device(fn_ctx, &fn_dev, 0);
	if (ret < 0) {
		freenect_shutdown(fn_ctx);
		return ret;
	}

	freenect_start_depth (fn_dev);
	freenect_start_video (fn_dev);

	freenect_registration regst = freenect_copy_registration(fn_dev);

	_pKinectV1Params->RefPixelSize = regst.zero_plane_info.reference_pixel_size;
	_pKinectV1Params->RefDistance  = regst.zero_plane_info.reference_distance;
    _pCameraParams->SetKinectV1ParametersInstance(_pKinectV1Params);

	freenect_frame_mode oDepthFrameMode = freenect_get_current_depth_mode(fn_dev);
	oDepthFrameMode.depth_format = FREENECT_DEPTH_REGISTERED;
	oDepthFrameMode.resolution 	 = FREENECT_RESOLUTION_MEDIUM;
	freenect_set_depth_mode(fn_dev, oDepthFrameMode);

	freenect_frame_mode oVideoFrameMode = freenect_get_current_video_mode(fn_dev);
	oVideoFrameMode.video_format = FREENECT_VIDEO_RGB;
	oVideoFrameMode.resolution 	 = FREENECT_RESOLUTION_MEDIUM;
	freenect_set_video_mode(fn_dev, oVideoFrameMode);

    oDepthFrameMode = freenect_get_current_depth_mode (fn_dev);
    oVideoFrameMode = freenect_get_current_video_mode (fn_dev);

    SetDepthFrameDimension(oDepthFrameMode.width, oDepthFrameMode.height);
    SetColorFrameDimension(oVideoFrameMode.width, oVideoFrameMode.height);

    freenect_stop_depth (fn_dev);
	freenect_stop_video (fn_dev);

	freenect_destroy_registration(&regst);
	freenect_close_device(fn_dev);
	freenect_shutdown(fn_ctx);
}

void KinectV1Sensor::Run() {
	_pDevice->GetCriticalSection().Lock();
	while (!_pDevice->IsColorFrameReceived() && !_pDevice->IsDepthFrameReceived())
	    _pDevice->GetCondition().ConditionalWait();

	if (_pDevice->IsColorFrameReceived()) {
		//
		// TODO : Copy the depth buffer
		//
	    _pDevice->SetColorFrameReceived(false);
	}

	if (_pDevice->IsDepthFrameReceived()) {
		//
		// TODO : Copy the RGB buffer
		//
	    _pDevice->SetDepthFrameReceived(false);
	}
	_oThreadSemaphore.SemaphorePost();
	_pDevice->GetCriticalSection().Unlock();
}