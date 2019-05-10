#include <stdio.h>

#include "Common.h"
#include "CameraParameters.h"
#include "CriticalSection.h"
#include "PCLUtils.h"
#include "OpenCVUtils.h"
#include "ThreadSyncSemaphore.h"
#include "DepthCameraReader.h"
#include "OpenNIDepthSensor.h"
#include "KinectV1Sensor.h"
#include "Utils.h"

DepthCameraReader::DepthCameraReader(IDepthSensorBuilder* pDepthSensorBuilder) : 
	_pDepthSensorBuilder(NULL) {
	_oDepthBufferInfo.pDepthBuf = NULL;
	_oColorBufferInfo.pColorBuf = NULL;
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
			_oDepthBufferInfo.pDepthBuf = _pDepthSensorBuilder->GetDepthBuffer();
			_oColorBufferInfo.pColorBuf = _pDepthSensorBuilder->GetColorBuffer();
			_oDepthBufferInfo.pFrameDim = _pDepthSensorBuilder->GetDepthFrameDimension();
			_oColorBufferInfo.pFrameDim = _pDepthSensorBuilder->GetColorFrameDimension();
			
			if (_oDepthBufferInfo.pDepthBuf == NULL) {
				printf ("Depth buffer is NULL\n");
				break;
			}
			if (_oColorBufferInfo.pColorBuf == NULL) {
				printf ("Color buffer is NULL\n");
				break;
			}

			OpenCVUtils oOpenCVUtils(&_oDepthBufferInfo, &_oColorBufferInfo);
			if (_pDepthSensorBuilder->GetCameraParameters()->GetCameraParameterType() == CAMERA_FOV_PARAMETERS) {
				PCLUtils oPCLUtils(	_oDepthBufferInfo.pFrameDim, _pDepthSensorBuilder->GetCameraParameters()->GetFovParametersInstance(), 
									_oDepthBufferInfo.pDepthBuf, _oColorBufferInfo.pColorBuf);
				oPCLUtils.GeneratePCDFileUsingFoVParams();
			} else if (_pDepthSensorBuilder->GetCameraParameters()->GetCameraParameterType() == CAMERA_INTRINSIC_PARAMETERS) {
				PCLUtils oPCLUtils(	_oDepthBufferInfo.pFrameDim, _pDepthSensorBuilder->GetCameraParameters()->GetIntrinsicParametersInstance(), 
									_oDepthBufferInfo.pDepthBuf, _oColorBufferInfo.pColorBuf);
				oPCLUtils.GeneratePCDFileUsingIntrinsicParams();
			} else if (_pDepthSensorBuilder->GetCameraParameters()->GetCameraParameterType() == CAMERA_KINECTV1_PARAMETERS) {
				PCLUtils oPCLUtils(	_oDepthBufferInfo.pFrameDim, _pDepthSensorBuilder->GetCameraParameters()->GetKinectV1ParametersInstance(), 
									_oDepthBufferInfo.pDepthBuf, _oColorBufferInfo.pColorBuf);
				oPCLUtils.GeneratePCDFileUsingKinectV1Parameters();
			} else {
				printf ("ERROR : Invalid Camera parameters. Unable to create PCD file.\n");
			}
			oOpenCVUtils.StoreDepthBufferAsImage(Utils::PrepareUniqueFileName("png", "png"));
			oOpenCVUtils.StoreColorBufferAsImage(Utils::PrepareUniqueFileName("jpg", "jpg"));
		}
	}
}

int main(int argc, char** argv) {
/*
	OpenNIDepthSensor oOpenNIDepthSensor;
	DepthCameraReader oDepthCameraReaderOpenNI(&oOpenNIDepthSensor);
	oDepthCameraReaderOpenNI.StartThread();
*/
	KinectV1Sensor oKinectV1Sensor;
	DepthCameraReader oDepthCameraReaderKinectV1Sensor(&oKinectV1Sensor);
	oDepthCameraReaderKinectV1Sensor.StartThread();

	//
	// Call blocking API here...
	//
	printf ("Press any key to quit the application...\n");
	char c = getchar();

	printf("Thread clean up begins...\n");
/*
	oOpenNIDepthSensor.StopThread();
	oDepthCameraReaderOpenNI.StopThread();

	oOpenNIDepthSensor.WaitForThreadToExit();
	oDepthCameraReaderOpenNI.WaitForThreadToExit();
*/	
	oKinectV1Sensor.StopThread();
	oDepthCameraReaderKinectV1Sensor.StopThread();

	oKinectV1Sensor.WaitForThreadToExit();
	oDepthCameraReaderKinectV1Sensor.WaitForThreadToExit();

	printf("Thread clean up ends...\n");

	return 0;
}
