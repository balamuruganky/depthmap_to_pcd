#include <stdio.h>

#include "Common.h"
#include "CameraParameters.h"
#include "CriticalSection.h"
#include "PCLUtils.h"
#include "OpenCVUtils.h"
#include "ThreadSyncSemaphore.h"
#include "DepthCameraReader.h"
#include "OpenNIDepthSensor.h"
#include "Utils.h"

DepthCameraReader::DepthCameraReader(IDepthSensorBuilder* pDepthSensorBuilder) : 
	_pDepthSensorBuilder(NULL), _pDepthBuffer(NULL), _pColorBuffer(NULL), _pDepthBufferDimension(NULL), _pColorBufferDimension(NULL) {
	_pDepthSensorBuilder = pDepthSensorBuilder;

	if (NULL != _pDepthSensorBuilder) {
		_pDepthSensorBuilder->InitDepthSensor();
	}
}

DepthCameraReader::~DepthCameraReader() {
	_pDepthSensorBuilder->DeInitDepthSensor();
}

void DepthCameraReader::Run() {
	while(IsThreadStart()) {
		_pDepthSensorBuilder->WaitForBufferStreams(5);
		{
			printf ("*** Depth and Color buffer is available ***\n");
			_pDepthBuffer = _pDepthSensorBuilder->GetDepthBuffer();
			_pColorBuffer = _pDepthSensorBuilder->GetColorBuffer();
			_pDepthBufferDimension = _pDepthSensorBuilder->GetDepthFrameDimension();
			_pColorBufferDimension = _pDepthSensorBuilder->GetColorFrameDimension();
			
			OpenCVUtils oOpenCVUtils(_pDepthBufferDimension, _pDepthBuffer, _pColorBufferDimension, _pColorBuffer);
			
			if (_pDepthSensorBuilder->GetCameraParameters()->GetCameraParameterType() == CAMERA_FOV_PARAMETERS) {
				PCLUtils oPCLUtils(	_pDepthBufferDimension, _pDepthSensorBuilder->GetCameraParameters()->GetFovParametersInstance(), 
									_pDepthBuffer, _pColorBuffer);
				oPCLUtils.GeneratePCDFileUsingFoVParams();
			} else if (_pDepthSensorBuilder->GetCameraParameters()->GetCameraParameterType() == CAMERA_INTRINSIC_PARAMETERS) {
				PCLUtils oPCLUtils(	_pDepthBufferDimension, _pDepthSensorBuilder->GetCameraParameters()->GetIntrinsicParametersInstance(), 
									_pDepthBuffer, _pColorBuffer);
				oPCLUtils.GeneratePCDFileUsingIntrinsicParams();
			} else {
				printf ("ERROR : Invalid Camera parameters. Unable to create PCD file.\n");
			}
			
			oOpenCVUtils.StoreDepthBufferAsImage(Utils::PrepareUniqueFileName("png", "png"));
			oOpenCVUtils.StoreColorBufferAsImage(Utils::PrepareUniqueFileName("jpg", "jpg"));
		}
	}
}

int main(int argc, char** argv) {
	OpenNIDepthSensor oOpenNIDepthSensor;
	DepthCameraReader oDepthCameraReader(&oOpenNIDepthSensor);

	oDepthCameraReader.StartThread();

	//
	// Call blocking API here...
	//
	char c = getchar();

	printf("Thread clean up begins...\n");

	oOpenNIDepthSensor.StopThread();
	oDepthCameraReader.StopThread();

	oOpenNIDepthSensor.WaitForThreadToExit();
	oDepthCameraReader.WaitForThreadToExit();

	printf("Thread clean up ends...\n");

	return 0;
}