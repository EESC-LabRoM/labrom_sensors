/*************************************************************************
*   Image pre processing.
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

// ROS libraries
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// ROS messages
#include "sensor_msgs/Image.h"

//! Top-level namespace
namespace labrom_sensors{
namespace camera{
#define IMAGE_UNKNOWN_ENCODING -1
#define IMAGE_MONO_8 0
#define IMAGE_RGB_8  1
class ImageProc{
  public:
    //! Constructor
    ImageProc(void);
    //! Destructor
    ~ImageProc(void);
    //! Subscriber
    void ImageCallback(const sensor_msgs::Image::ConstPtr &msg);
    //! Set encoding 
    void SetEncoding(std::string encoding);
  private:
    ros::NodeHandle nh_;                       //!< ROS node handle (private)
    ros::NodeHandle node_;                     //!< ROS node handle

    image_transport::Subscriber image_sub_;    //!< Input image subscriber
    image_transport::Publisher  image_pub_;    //!< Input image subscriber

    int encoding_;                             //!< Image encoding
};
} // camera namespace
} // labrom_sensors namespace