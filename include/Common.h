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

typedef struct {
	uint8_t Red;
	uint8_t Green;
	uint8_t Blue;
} ColorBuffer;

enum CameraParametersType {
	CAMERA_FOV_PARAMETERS,
	CAMERA_INTRINSIC_PARAMETERS,
	CAMERA_KINECTV1_PARAMETERS,
	// Invalid parameter
	CAMERA_PARAMETER_INVALID
};

typedef struct {
	int Width;
	int Height;
} FrameDimension;

typedef struct {
	float DepthHorizontalFOV;
	float DepthVerticalFOV;
	float ColorHorizontalFOV;
	float ColorVerticalFOV;
} FOVParameters;

typedef struct {
	// Focal lengths
	float Fx;
	float Fy;
	// Principle points
	float Cx;
	float Cy;
} IntrinsicParameters;

typedef struct {
	float RefPixelSize;
	float RefDistance;
}KinectV1Parameters;

#endif