#include <Serializer.h>

namespace boost
{
    namespace serialization
    {
	    template<class Archive>
	    void serialize(Archive & ar, Serializer::DepthColorInfo &d, const unsigned int file_version) {
	        ar & d.timestamp;
	        if (Archive::is_loading::value) {
	            d.oDepthBufferInfo.pFrameDim = new Serializer::FrameDimension;
	        }
	        ar & d.oDepthBufferInfo.pFrameDim->Width;
	        ar & d.oDepthBufferInfo.pFrameDim->Height;
	      	if (Archive::is_loading::value) {
	            d.oDepthBufferInfo.pDepthBuf = new Serializer::DepthBuffer[d.oDepthBufferInfo.pFrameDim->Width * d.oDepthBufferInfo.pFrameDim->Height];
	        }
	        ar & boost::serialization::make_array<Serializer::DepthBuffer>(d.oDepthBufferInfo.pDepthBuf, (d.oDepthBufferInfo.pFrameDim->Width * d.oDepthBufferInfo.pFrameDim->Height));

	       	if (Archive::is_loading::value) {
	            d.oColorBufferInfo.pFrameDim = new Serializer::FrameDimension;
	        }
	        ar & d.oColorBufferInfo.pFrameDim->Width;
	        ar & d.oColorBufferInfo.pFrameDim->Height;
	        if (Archive::is_loading::value) {
	            d.oColorBufferInfo.pColorBuf = new Serializer::ColorBuffer[d.oColorBufferInfo.pFrameDim->Width * d.oColorBufferInfo.pFrameDim->Height];
	        }
	        ar & boost::serialization::make_binary_object(d.oColorBufferInfo.pColorBuf, (d.oColorBufferInfo.pFrameDim->Width * d.oColorBufferInfo.pFrameDim->Height));
	    }
    }
}

namespace Serializer {
	void SerializeDepthColorInfo(DepthColorInfo &depthColorInfo, std::string FileName)
	{
		// serialize structure
		{
			std::ofstream ofs_depthcolor(FileName.c_str());
			boost::archive::text_oarchive oa_depthcolor(ofs_depthcolor);
			oa_depthcolor & depthColorInfo;
		}
	}

	void DeserializeDepthColorInfo(std::string FileName, DepthColorInfo &depthColorInfo)
	{
		// deserialize structure
		{
			std::ifstream ifs_depthcolor(FileName.c_str());
			boost::archive::text_iarchive ia_depthcolor(ifs_depthcolor);
			ia_depthcolor & depthColorInfo;
		}
	}
}
