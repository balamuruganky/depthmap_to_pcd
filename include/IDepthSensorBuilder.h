#ifndef IDEPTH_SENSOR_H
#define IDEPTH_SENSOR_H

#include "Common.h"
#include "CameraParameters.h"

class IDepthSensorBuilder
{
public:
  virtual ~ IDepthSensorBuilder ()
  {
  }
  virtual int8_t InitDepthSensor () = 0;
  virtual int8_t DeInitDepthSensor () = 0;
  virtual int32_t WaitForBufferStreams (uint16_t TimeOutInSeconds) = 0;

  virtual DepthBuffer *GetDepthBuffer () = 0;
  virtual FrameDimension *GetDepthFrameDimension () = 0;

  virtual ColorBuffer *GetColorBuffer () = 0;
  virtual FrameDimension *GetColorFrameDimension () = 0;

  virtual CameraParameters *GetCameraParameters () = 0;
};

#endif
