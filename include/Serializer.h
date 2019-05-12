#ifndef SERIALISER_H_
#define SERIALISER_H_

#include <iostream>
#include <fstream>
#include <stdint.h>

// include input and output archivers
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

// include this header to serialize vectors
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/binary_object.hpp>

using namespace std;

namespace Serializer {
	typedef uint16_t DepthBuffer;

	typedef struct
	{
		uint8_t Red;
		uint8_t Green;
		uint8_t Blue;
	} ColorBuffer;

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

	typedef struct {
		// Header
	    uint32_t 		timestamp;
	    DepthBufferInfo oDepthBufferInfo;
	    ColorBufferInfo oColorBufferInfo;
	} DepthColorInfo;

	void SerializeDepthColorInfo(DepthColorInfo &depthColorInfo, std::string FileName);
	void DeserializeDepthColorInfo(std::string FileName, DepthColorInfo &depthColorInfo);

}

#endif /* SERIALISER_H_ */