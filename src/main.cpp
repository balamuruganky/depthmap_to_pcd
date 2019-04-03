#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include <iostream>
#include <time.h>
#include <math.h>
using namespace std;

#include <OpenNI.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class Grabber
{
public:
    void  InitOpenNI();
    void  InitDevice();
    void  InitDepthStream();
    void  InitColorStream();
    void  Run();

private:
    void CapturePsenseDepthFrame();
    void CapturePsenseColorFrame();
    cv::Mat ChangeDepthForDisplay(const cv::Mat& mat);
    float GetDepthStreamFocalLength();
    void SaveAsPCDFile();
    char* GetFileName();

    openni::Device*        device_;
    openni::VideoStream*   depth_stream_;
    openni::VideoStream*   color_stream_;
    openni::VideoFrameRef* depth_frame_;
    openni::VideoFrameRef* color_frame_;
    openni::RGB888Pixel*   dev_rgbbuf_ptr_;
    openni::DepthPixel*    dev_depthbuf_ptr_;
};

char* Grabber::GetFileName() {
  time_t rawtime;
  struct tm * timeinfo;
  static char buffer [80];

  time (&rawtime);
  timeinfo = localtime (&rawtime);

  strftime (buffer,80,"/home/pi/pcd/%F_%I_%M%p_%S.pcd",timeinfo);
  
  return buffer;
}

void Grabber::InitOpenNI()
{
    auto rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK)
    {
        printf("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
        exit(0);
    }
}

void Grabber::InitDevice()
{
    device_ = new openni::Device();
    auto rc = device_->open(openni::ANY_DEVICE);
    if (rc != openni::STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", openni::OpenNI::getExtendedError());
        exit(0);
    }
}

void Grabber::InitDepthStream()
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
            vmode.getResolutionX() == 640 &&
            vmode.getResolutionY() == 480)
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
}

void Grabber::InitColorStream()
{
    color_stream_ = new openni::VideoStream();

    if (device_->getSensorInfo(openni::SENSOR_COLOR) != nullptr)
    {
        auto rc = color_stream_->create(*device_, openni::SENSOR_COLOR);
        if (rc != openni::STATUS_OK)
        {
            printf("Couldn't create color stream\n%s\n", openni::OpenNI::getExtendedError());
            exit(0);
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
            vmode.getResolutionX() == 640 &&
            vmode.getResolutionY() == 480)
        {
            color_stream_->setVideoMode(vmode);
            break;
        }
    }
    
    device_->setDepthColorSyncEnabled(true);

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

    // Start color stream
    auto rc = color_stream_->start();
    if (rc != openni::STATUS_OK)
    {
        printf("Couldn't start the depth stream\n%s\n", openni::OpenNI::getExtendedError());
        exit(0);
    }

    color_frame_ = new openni::VideoFrameRef();
}

void Grabber::CapturePsenseDepthFrame()
{
    auto rc = depth_stream_->readFrame(depth_frame_);
    if (rc != openni::STATUS_OK)
    {
        printf("Read failed!\n%s\n", openni::OpenNI::getExtendedError());
    }

    if (depth_frame_->getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM && depth_frame_->getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_100_UM)
    {
        printf("Unexpected frame format\n");
    }

    // Get pointer to Primesense depth frame
    dev_depthbuf_ptr_ = (openni::DepthPixel*) depth_frame_->getData();

    // Copy frame data to OpenCV mat
    cv::Mat depth_mat(depth_frame_->getHeight(), depth_frame_->getWidth(), CV_16U, dev_depthbuf_ptr_);

    cv::Mat disp_mat = ChangeDepthForDisplay(depth_mat);

    cv::imshow("Depth", disp_mat);
}

float Grabber::GetDepthStreamFocalLength() { 
   float focal_length = 0.0f;
   focal_length = (float)depth_frame_->getWidth() / (2 * tan(depth_stream_->getHorizontalFieldOfView() / 2));
   //cout << "Focal Length : " << focal_length << endl;
   return focal_length;
}

void Grabber::SaveAsPCDFile() {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pointcloud->width = depth_frame_->getWidth();
  pointcloud->height = depth_frame_->getHeight();
  pointcloud->points.resize (pointcloud->height * pointcloud->width);
  
  // Make mat from camera data
   cv::Mat color_mat(color_frame_->getHeight(), color_frame_->getWidth(), CV_8UC3, dev_rgbbuf_ptr_);
   // Convert to BGR format for OpenCV
   cv::cvtColor(color_mat, color_mat, CV_RGB2BGR);
  
  unsigned depth_idx = 0;
    for (unsigned v = 0; v < depth_frame_->getHeight(); ++v) {
      for (unsigned u = 0; u < depth_frame_->getWidth(); ++u, ++depth_idx)  {
        pcl::PointXYZRGB& pt = pointcloud->points[depth_idx]; 
        pt.z = dev_depthbuf_ptr_[depth_idx] * 0.001f;
        
        int r_i = depth_idx / (int)depth_frame_->getWidth();
        int c_i = depth_idx % (int)depth_frame_->getWidth();
        
        //
        // Calculate X coordinate value
        //
        float alpha_h = (M_PI - depth_stream_->getHorizontalFieldOfView()) / 2;
        float gamma_i_h = alpha_h + (float)c_i*(depth_stream_->getHorizontalFieldOfView() / depth_frame_->getWidth());
        pt.x = pt.z / tan(gamma_i_h);
        
        //
        // Calculate Y coordinate value
        //
        float alpha_v = 2 * M_PI - (depth_stream_->getVerticalFieldOfView() / 2);
        float gamma_i_v = alpha_v + (float)r_i*(depth_stream_->getVerticalFieldOfView() / depth_frame_->getHeight());
        pt.y = pt.z * tan(gamma_i_v)*-1;

        //
        // Assign colour values
        //
        pt.r = color_mat.at<cv::Vec3b>(v,u)[0];
        pt.g = color_mat.at<cv::Vec3b>(v,u)[1];
        pt.b = color_mat.at<cv::Vec3b>(v,u)[2];
      }
  }
  
  pointcloud->sensor_origin_.setZero (); 
  pointcloud->sensor_orientation_.w () = 0.0f;
  pointcloud->sensor_orientation_.x () = 0.0f;
  pointcloud->sensor_orientation_.y () = 1.0f;
  pointcloud->sensor_orientation_.z () = 0.0f;
  
  pcl::io::savePCDFile (GetFileName(), *pointcloud);
}

/*
void Grabber::SaveAsPCDFile() {
    //pcl::PointCloud<pcl::PointXYZ> pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pointcloud->width = depth_frame_->getWidth();
  pointcloud->height = depth_frame_->getHeight();
  pointcloud->points.resize (pointcloud->height * pointcloud->width);

   //cout << "Width : " << pointcloud->width << ", Height : " << pointcloud->height << endl;
  //int* depth_data = new int[pointcloud.height * pointcloud.width]; 
  //copy the depth values of every pixel in here 

  register float constant = 1.0f / GetDepthStreamFocalLength();
  register int centerX = (pointcloud->width >> 1); 
  int centerY = (pointcloud->height >> 1); 
  register int depth_idx = 0;
  for (int v = -centerY; v < centerY; ++v) 
  { 
    for (register int u = -centerX; u < centerX; ++u, ++depth_idx) 
    { 
      pcl::PointXYZRGB& pt = pointcloud->points[depth_idx]; 
      pt.z = dev_depthbuf_ptr_[depth_idx] * 0.001f; 
      pt.x = static_cast<float> (u) * pt.z * constant; 
      pt.y = static_cast<float> (v) * pt.z * constant;
      int red = dev_rgbbuf_ptr_[depth_idx].r&0xff;
      int green = dev_rgbbuf_ptr_[depth_idx].g&0xff;
      int blue = dev_rgbbuf_ptr_[depth_idx].b&0xff;
         uint32_t rgb = (red << 16 | green << 8 | blue);
         //uint32_t rgb = ((uint32_t)dev_rgbbuf_ptr_[depth_idx].r << 16 | (uint32_t)dev_rgbbuf_ptr_[depth_idx].g << 8 | (uint32_t)dev_rgbbuf_ptr_[depth_idx].b);
      pt.rgba = rgb;
                        //pt.rgb = *reinterpret_cast<float*>(&rgb);
      //pt.r = dev_rgbbuf_ptr_[depth_idx].r;
      //pt.g = dev_rgbbuf_ptr_[depth_idx].g;
      //pt.b = dev_rgbbuf_ptr_[depth_idx].b;
      //cout << pt.r << ", " << pt.g << ", " << pt.b << endl;
      //printf ("Colour value : %d %d %d\n", pt.r, pt.g, pt.b);
    } 
  } 
  pointcloud->sensor_origin_.setZero (); 
  pointcloud->sensor_orientation_.w () = 0.0f; 
  pointcloud->sensor_orientation_.x () = 0.0f; 
  pointcloud->sensor_orientation_.y () = 0.0f; 
  pointcloud->sensor_orientation_.z () = 1.0f; 
  
  pcl::io::savePCDFile (GetFileName(), *pointcloud);
}
*/

void Grabber::CapturePsenseColorFrame()
{
    // Read from stream to frame
    auto rc = color_stream_->readFrame(color_frame_);
    if (rc != openni::STATUS_OK)
    {
        printf("Read failed!\n%s\n", openni::OpenNI::getExtendedError());
    }

    // Pointer to Primesense color frame
    dev_rgbbuf_ptr_ = (openni::RGB888Pixel*) color_frame_->getData();

    // Make mat from camera data
    cv::Mat color_mat(color_frame_->getHeight(), color_frame_->getWidth(), CV_8UC3, dev_rgbbuf_ptr_);
    // Convert to BGR format for OpenCV
    cv::cvtColor(color_mat, color_mat, CV_RGB2BGR);

    cv::imshow("Color", color_mat);
}

void Grabber::Run()
{
    openni::VideoStream* streams[] = {depth_stream_, color_stream_};
    bool isDepthAvailable = false, isRGBAvailable = false;

   while (true)
   {
       int readyStream = -1;
       auto rc = openni::OpenNI::waitForAnyStream(streams, 2, &readyStream, 2000);
       if (rc != openni::STATUS_OK)
       {
           printf("Wait failed! (timeout is %d ms)\n%s\n", 2000, openni::OpenNI::getExtendedError());
           break;
       }

       switch (readyStream)
       {
       case 0:
        //cout << "Before CapturePsenseDepthFrame" << endl;
        CapturePsenseDepthFrame();
        isDepthAvailable = true;
        //cout << "After CapturePsenseDepthFrame" << endl;
           break;
       case 1:
        //cout << "Before CapturePsenseColorFrame" << endl;
        CapturePsenseColorFrame();
        isRGBAvailable = true;
        //cout << "After CapturePsenseColorFrame" << endl;
           break;
       default:
           printf("Unxpected stream\n");
       }
       
       //
       // Store a PCD file
       //
       if (isDepthAvailable && isRGBAvailable) {
        //cout << "Before SaveAsPCDFile" << endl;
        SaveAsPCDFile();
        isDepthAvailable = isRGBAvailable = false;
        //cout << "After SaveAsPCDFile" << endl;
       }

       char c = cv::waitKey(10);
       if ('q' == c)
           break;
   }
}

cv::Mat Grabber::ChangeDepthForDisplay(const cv::Mat& mat)
{
    assert(CV_16U == mat.type());

    const float depth_near = 500;
    const float depth_far  = 5000;

    const float alpha = 255.0 / (depth_far - depth_near);
    const float beta  = - depth_near * alpha;

    cv::Mat fmat;
    mat.convertTo(fmat, CV_32F);

    for (int r = 0; r < mat.rows; ++r)
    {
        for (int c = 0; c < mat.cols; ++c)
        {
            float v = fmat.at<float>(r, c) * alpha + beta;
            
            if (v > 255) v = 255;
            if (v < 0)   v = 0;

            fmat.at<float>(r, c) = v;
        }
    }

    cv::Mat bmat;
    fmat.convertTo(bmat, CV_8U);

    cv::Mat cmat;
    cv::cvtColor(bmat, cmat, CV_GRAY2BGR);
    cv::applyColorMap(cmat, cmat, cv::COLORMAP_OCEAN);

    return cmat;
}

/*
float CalculateFocalLength(int8_t WidthOrHeight, float HFovOrVFov) {
  float focal_length = 0.0f;
  focal_length = (float)WidthOrHeight / (2 * tan(HFovOrVFov / 2));
  return focal_length;
}
*/

/*
int main()
{
    Grabber grabber;
    grabber.InitOpenNI();
    grabber.InitDevice();
    grabber.InitDepthStream();
    grabber.InitColorStream();
    grabber.Run();

    return 0;
}
*/