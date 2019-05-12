#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <iostream>
#include <time.h>
#include <math.h>
using namespace std;

#define SUCCESS 0
#define ERROR 	-1

typedef uint16_t DepthBuffer;

typedef struct
{
  uint8_t Red;
  uint8_t Green;
  uint8_t Blue;
} ColorBuffer;

typedef enum
{
  CAMERA_FOV_PARAMETERS,
  CAMERA_INTRINSIC_PARAMETERS,
  CAMERA_KINECTV1_PARAMETERS,
  // Invalid parameter
  CAMERA_PARAMETER_INVALID
} CameraParametersType;

typedef struct
{
  int Width;
  int Height;
} FrameDimension;

typedef struct
{
  DepthBuffer *pDepthBuf;
  FrameDimension *pFrameDim;
} DepthBufferInfo;

typedef struct
{
  ColorBuffer *pColorBuf;
  FrameDimension *pFrameDim;
} ColorBufferInfo;

#endif
