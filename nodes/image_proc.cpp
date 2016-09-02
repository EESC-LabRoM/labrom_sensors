/*************************************************************************
*   Image pre processing implementation
*   This file is part of labrom_sensors
*
*   labrom_sensors is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_sensors is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_sensors.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

// labrom_sensors libraries
#include "labrom_sensors/image_proc.h"

namespace labrom_sensors{
namespace camera{
/**
* Constructor
*/
ImageProc::ImageProc(void): pnh_("~"){
  // parameters
  std::string encoding;
  pnh_.getParam("encoding",encoding);

  // Setting parameters
  SetEncoding(encoding);

  // Initialize transport
  image_transport::ImageTransport it(nh_);
  image_transport::ImageTransport pit(pnh_);  
  // Publishers and subscribers
  image_sub_ = it.subscribe("image_raw",1,&ImageProc::ImageCallback, this);
  image_pub_  = pit.advertise("image_mono",1);  
}

/**
* Destructor
*/ 
ImageProc::~ImageProc(void){};

/**
* Image callback. Do some pre-processment if required: rgb to grayscale
*/
void ImageProc::ImageCallback(const sensor_msgs::Image::ConstPtr &msg){
  try{
    // 1: Converting image from ROS to OpenCV format   
    cv::Mat img;
    cv_bridge::CvImagePtr cvImagePtr     = cv_bridge::toCvCopy(msg);
    img = cvImagePtr->image;
    // 2: Converting to grayscale
    cv::Mat gray;

    switch(encoding_){
      case (IMAGE_MONO_8):
        gray = img;
        break;
      case (IMAGE_RGB_8):
        cv::cvtColor( img, gray, CV_RGB2GRAY);
        break;
      default:
        ROS_WARN("Unknown encoding"); 
    }

      
    // 3: Publish image
    sensor_msgs::ImagePtr image_msg  = cv_bridge::CvImage(msg->header, "mono8", gray ).toImageMsg();
    image_pub_.publish(image_msg);

  }catch (cv_bridge::Exception &e){
    ROS_ERROR("CV_BRIDGE Exception: %s", e.what());
  }     

}

/**
* Sets image encoding.
* @param[in] encoding string that contains image encoding
*/
void ImageProc::SetEncoding(std::string encoding){
  if(encoding == "mono8"){
    encoding_ = IMAGE_MONO_8;
  } else if(encoding == "rgb8"){
    encoding_ = IMAGE_RGB_8;
  } else{
    encoding_ = IMAGE_UNKNOWN_ENCODING;
  }
}

} // camera namespace
} // labrom_sensors namespace

int main(int argc, char** argv){
  ros::init(argc,argv,"image_demux");
  
  labrom_sensors::camera::ImageProc proc;

  ros::spin();

}
