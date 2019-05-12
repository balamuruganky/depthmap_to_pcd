#include <OpenNI.h>
#include <unistd.h>

#include "OpenNIDepthSensor.h"
#include "CriticalSection.h"

#define STREAM_WIDTH	640
#define STREAM_HEIGHT	480

OpenNIDepthSensor::OpenNIDepthSensor() : _pCameraParams(NULL), _pFovParams(NULL) {
	_pCameraParams = new CameraParameters();
	_pFovParams = new FOVParameters();
	
	_oDepthFrameDimension = _oColorFrameDimension = {0,0};
}

OpenNIDepthSensor::~OpenNIDepthSensor() {
	delete _pFovParams;
	delete _pCameraParams;
}

int8_t OpenNIDepthSensor::InitDepthSensor() {
	int8_t ret = 0;
    if (SUCCESS != InitOpenNI()) {
    	printf("InitOpenNI failed!!!");
    	return ERROR;
    } 
    if (SUCCESS != InitDevice()) {
    	printf("InitDevice failed!!!");
    	return ERROR;
    }
    if (SUCCESS != InitDepthStream()) {
    	printf("InitDepthStream failed!!!");
    	return ERROR;
    }
    if (SUCCESS != InitColorStream()) {
    	printf("InitColorStream failed!!!");
    	return ERROR;
    }

    SetCameraParams();

    StartThread();

    return ret;
}

int8_t OpenNIDepthSensor::DeInitDepthSensor() {
	DeInitDevice();
    DeInitOpenNI();
    // I knew, it is stupid.
	return SUCCESS;
}

FrameDimension* OpenNIDepthSensor::GetDepthFrameDimension() {
	return (&_oDepthFrameDimension);
}

void OpenNIDepthSensor::SetDepthFrameDimension() {
	if (depth_frame_ != NULL) {
		_oDepthFrameDimension.Width 	= (int)depth_frame_->getWidth();
		_oDepthFrameDimension.Height 	= (int)depth_frame_->getHeight();
	}
}

FrameDimension* OpenNIDepthSensor::GetColorFrameDimension() {
	return (&_oColorFrameDimension);
}

void OpenNIDepthSensor::SetColorFrameDimension() {
	if (color_frame_ != NULL) {
		_oColorFrameDimension.Width 	= (int)color_frame_->getWidth();
		_oColorFrameDimension.Height 	= (int)color_frame_->getHeight();
	}
}

CameraParameters* OpenNIDepthSensor::GetCameraParameters() {
	return _pCameraParams;
}

DepthBuffer* OpenNIDepthSensor::GetDepthBuffer() {
	return dev_depthbuf_ptr_;
}

ColorBuffer* OpenNIDepthSensor::GetColorBuffer() {
	return ((ColorBuffer*)dev_rgbbuf_ptr_);
}

void OpenNIDepthSensor::SetCameraParams() {
	_pFovParams->SetDepthHorizontalFOV(depth_stream_->getHorizontalFieldOfView());
    _pFovParams->SetDepthVerticalFOV(depth_stream_->getVerticalFieldOfView());
	_pFovParams->SetColorHorizontalFOV(color_stream_->getHorizontalFieldOfView());
    _pFovParams->SetColorVerticalFOV(color_stream_->getVerticalFieldOfView());
    _pCameraParams->SetFovParametersInstance(_pFovParams);
}

int32_t	OpenNIDepthSensor::WaitForBufferStreams(uint16_t TimeOutInSeconds) {
	return _oThreadSemaphore.SemaphoreTimedWait(TimeOutInSeconds);
}

void OpenNIDepthSensor::Run() {
	openni::VideoStream* streams[] = {depth_stream_, color_stream_};
	bool isDepthAvailable = false, isRGBAvailable = false;

	CriticalSection oCriticalSection;
	while (IsThreadStart()) {
		int readyStream = -1;
		auto rc = openni::OpenNI::waitForAnyStream(streams, 2, &readyStream, 2000);
		if (rc != openni::STATUS_OK) {
			printf("Wait failed! (timeout is %d ms)\n%s\n", 2000, openni::OpenNI::getExtendedError());
			break;
		}
		
		switch (readyStream) {
			case 0:
				oCriticalSection.Lock();
				CapturePsenseDepthFrame();
				isDepthAvailable = true;
				oCriticalSection.Unlock();
				break;
			case 1:
				oCriticalSection.Lock();
				CapturePsenseColorFrame();
				isRGBAvailable = true;
				oCriticalSection.Unlock();
				break;
			default:
				printf("Error : unexpected stream\n");
		}

		if (isDepthAvailable && isRGBAvailable) {
			_oThreadSemaphore.SemaphorePost();
			isDepthAvailable = isRGBAvailable = false;
		}
		sleep (1);
	}
}

void OpenNIDepthSensor::DeInitOpenNI() {
	openni::OpenNI::shutdown();
}

void OpenNIDepthSensor::DeInitDevice() {
	depth_frame_->release();
	color_frame_->release();	
	
	depth_stream_->stop();
	color_stream_->stop();

	depth_stream_->destroy();
	color_stream_->destroy();
	
	device_->close();
}

int8_t OpenNIDepthSensor::InitOpenNI()
{
    auto rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK)
    {
        printf("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
        return ERROR;
    }
    return SUCCESS;
}

int8_t OpenNIDepthSensor::InitDevice()
{
    device_ = new openni::Device();
    auto rc = device_->open(openni::ANY_DEVICE);
    if (rc != openni::STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", openni::OpenNI::getExtendedError());
        return ERROR;
    }
    return SUCCESS;
}

int8_t OpenNIDepthSensor::InitDepthStream()
{
    depth_stream_ = new openni::VideoStream();

    // Create depth stream from device
    if (device_->getSensorInfo(openni::SENSOR_DEPTH) != nullptr)
    {
        auto rc = depth_stream_->create(*device_, openni::SENSOR_DEPTH);
        if (rc != openni::STATUS_OK)
        {
            printf("Couldn't create depth stream\n%s\n", openni::OpenNI::getExtendedError());
            exit(0);
        }
    }

    // Get info about depth sensor
    const openni::SensorInfo& sensor_info       = *device_->getSensorInfo(openni::SENSOR_DEPTH);
    const openni::Array<openni::VideoMode>& arr = sensor_info.getSupportedVideoModes();

    // Look for VGA mode in depth sensor and set it for depth stream
    for (int i = 0; i < arr.getSize(); ++i)
    {
        const openni::VideoMode& vmode = arr[i];
        if (vmode.getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM &&
            vmode.getResolutionX() == STREAM_WIDTH &&
            vmode.getResolutionY() == STREAM_HEIGHT)
        {
            depth_stream_->setVideoMode(vmode);
            break;
        }
    }

    // Start the depth stream
    auto rc = depth_stream_->start();
    if (rc != openni::STATUS_OK)
    {
        printf("Couldn't start the depth stream\n%s\n", openni::OpenNI::getExtendedError());
        exit(0);
    }

    depth_frame_ = new openni::VideoFrameRef();
    return SUCCESS;
}

int8_t OpenNIDepthSensor::InitColorStream()
{
    color_stream_ = new openni::VideoStream();

    if (device_->getSensorInfo(openni::SENSOR_COLOR) != nullptr)
    {
        auto rc = color_stream_->create(*device_, openni::SENSOR_COLOR);
        if (rc != openni::STATUS_OK)
        {
            printf("Couldn't create color stream\n%s\n", openni::OpenNI::getExtendedError());
            return ERROR;
        }
    }

    // Get info about color sensor
    const openni::SensorInfo& sensor_info       = *device_->getSensorInfo(openni::SENSOR_COLOR);
    const openni::Array<openni::VideoMode>& arr = sensor_info.getSupportedVideoModes();

    // Look for VGA mode and set it for color stream
    for (int i = 0; i < arr.getSize(); ++i)
    {
        const openni::VideoMode& vmode = arr[i];
        if (
            vmode.getResolutionX() == STREAM_WIDTH &&
            vmode.getResolutionY() == STREAM_HEIGHT)
        {
            color_stream_->setVideoMode(vmode);
            break;
        }
    }

    // Note: Doing image registration earlier than this seems to fail
    if (device_->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
        auto rc = device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        if (rc == openni::STATUS_OK)
            std::cout << "Depth to color image registration set success\n";
        else
            std::cout << "Depth to color image registration set failed\n";
    }
    else
    {
        std::cout << "Depth to color image registration is not supported!!!\n";
    }

    device_->setDepthColorSyncEnabled(true);

    // Start color stream
    auto rc = color_stream_->start();
    if (rc != openni::STATUS_OK)
    {
        printf("Couldn't start the depth stream\n%s\n", openni::OpenNI::getExtendedError());
        return ERROR;
    }

    color_frame_ = new openni::VideoFrameRef();
    return SUCCESS;
}

void OpenNIDepthSensor::CapturePsenseDepthFrame()
{
    auto rc = depth_stream_->readFrame(depth_frame_);
    if (rc != openni::STATUS_OK) {
        printf("Read failed!\n%s\n", openni::OpenNI::getExtendedError());
    } else {
    	SetDepthFrameDimension();
    }

    if (depth_frame_->getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM && depth_frame_->getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_100_UM)
    {
        printf("Unexpected frame format\n");
    }

    // Get pointer to Primesense depth frame
    dev_depthbuf_ptr_ = (openni::DepthPixel*) depth_frame_->getData();
}

void OpenNIDepthSensor::CapturePsenseColorFrame()
{
    // Read from stream to frame
    auto rc = color_stream_->readFrame(color_frame_);
    if (rc != openni::STATUS_OK) {
        printf("Read failed!\n%s\n", openni::OpenNI::getExtendedError());
    } else {
    	SetColorFrameDimension();
    }

    // Pointer to Primesense color frame
    dev_rgbbuf_ptr_ = (openni::RGB888Pixel*) color_frame_->getData();
}
