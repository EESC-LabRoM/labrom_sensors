/*************************************************************************
*   Image demux. Implementation 
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
#include "labrom_sensors/image_demux.h"

namespace labrom_sensors{
/**
* Constructor
*/
ImageDemux::ImageDemux(void): nh_("~"){
  // parameters
  nh_.param<std::string>("encoding",_encoding,"rgb8");
  // Initialize transport
  image_transport::ImageTransport it(node_);
  // Publishers and subscribers
  image_sub_ = it.subscribe("image_raw",1,&ImageDemux::ImageCallback, this);
  image_pub_  = it.advertise("image_mono",1);  
}

/**
* Destructor
*/ 
ImageDemux::~ImageDemux(void){};

/**
* Image callback. Do some pre-processment if required: rgb to grayscale
*/
void ImageDemux::ImageCallback(const sensor_msgs::Image::ConstPtr &msg){
  try{
    // 1: Converting image from ROS to OpenCV format   
    cv::Mat img;
    cv_bridge::CvImagePtr cvImagePtr     = cv_bridge::toCvCopy(msg);
    img = cvImagePtr->image;
    // 2: Converting to grayscale
    cv::Mat gray;
    if(_encoding != "mono8")
      cv::cvtColor( img, gray, CV_RGB2GRAY);
    else
      gray = img;
      
    // 3: Publish image
    sensor_msgs::ImagePtr opt_flow_image  = cv_bridge::CvImage(msg->header, "mono8", gray ).toImageMsg();
    image_pub_.publish(opt_flow_image);

  }catch (cv_bridge::Exception &e){
    ROS_ERROR("CV_BRIDGE Exception: %s", e.what());
  }     
  
}
}

int main(int argc, char** argv){
  ros::init(argc,argv,"image_demux");
  
  labrom_sensors::ImageDemux demux;

  ros::spin();

}