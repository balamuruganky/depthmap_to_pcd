#ifndef DEPTH_CAMERA_READER_H
#define DEPTH_CAMERA_READER_H

#include "Common.h"
#include "IDepthSensorBuilder.h"
#include "IThread.h"

class DepthCameraReader:public IThread
{
public:
  DepthCameraReader (IDepthSensorBuilder * pDepthSensorBuilder);
  ~DepthCameraReader ();

private:
  IDepthSensorBuilder * _pDepthSensorBuilder;

  DepthBufferInfo _oDepthBufferInfo;
  ColorBufferInfo _oColorBufferInfo;
  void Run ();
};

#endif // DEPTH_CAMERA_READER_H
