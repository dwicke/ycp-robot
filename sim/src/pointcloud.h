
int pc_seq=0;

//#include "dynamic_reconfigure/ConfigDescription.h"
//#include "dynamic_reconfigure/Config.h"

//#include "tf/transform_broadcaster.h"
//#include "tf/transform_listener.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;




namespace enc = sensor_msgs::image_encodings;
typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

//NEED TO COMPUTE THIS MANUALLY!!
float pc_center_x=319.5,pc_center_y=239.5,
pc_constant_x=0.000002,pc_constant_y=0.000002;


/*
#include "sensor_msgs/CameraInfo.h"
#include <image_geometry/pinhole_camera_model.h>
image_geometry::PinholeCameraModel model_;
  // Use correct principal point from calibration
  model_.fromCameraInfo(info_msg);
  float center_x = model_.cx();
  float center_y = model_.cy();
  
    // Pre-compute constants for focal length and m->mm conversion
  float constant_x = 0.001 / model_.fx();
  float constant_y = 0.001 / model_.fy();
*/
//////////////////////////////////

  /*
void PointCloudXyzrgbNodelet__imageCb(const sensor_msgs::Image::ConstPtr& depth_msg,
                                      const sensor_msgs::Image::ConstPtr& rgb_msg,
                                      const sensor_msgs::CameraInfo::ConstPtr& info_msg)

*/


/*
	const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
	const uint8_t* rgb_buffer = &rgb_msg->data[0];
*/

//void send_pointcloud(const uint16_t* depth_data,const uint8_t* rgb_buffer)
void send_pointcloud(const float* depth_data,const uint8_t* rgb_buffer,float *depth_out_view_buffer,uint16_t *depth_out_raw_buffer,float *depth_out_float_buffer)
{
	//GL_RGB
	int red_offset, green_offset, blue_offset, color_step;

	red_offset   = 0;
	green_offset = 1;
	blue_offset  = 2;
	color_step   = 3;

	// Allocate new point cloud message
	PointCloud::Ptr cloud_msg (new PointCloud);

	//Populate header
	cloud_msg->header.seq=pc_seq++;
	cloud_msg->header.stamp=ros::Time::now();
	cloud_msg->header.frame_id=TF_POINTCLOUD_FRAME_ID;

	//Populate info
	cloud_msg->height = SENSOR_WINDOW_HEIGHT;
	cloud_msg->width  = SENSOR_WINDOW_WIDTH;
	cloud_msg->is_dense = false;
	cloud_msg->points.resize (cloud_msg->height * cloud_msg->width);
	
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	int depth_idx = 0, color_idx = 0;
	
	//Populate data
	PointCloud::iterator pt_iter = cloud_msg->begin();
	//for (int v = 0; v < (int)cloud_msg->height; ++v)
	
	//NOTE: This is done bottom-to-top, as GL renders that way.
	int i=0;
	for (int v = 0; v < (int)cloud_msg->height ; ++v)
	{
		for (int u = 0; u < (int)cloud_msg->width; ++u, ++depth_idx, color_idx += color_step, ++pt_iter)
		{
			pcl::PointXYZRGB& pt = *pt_iter;
			
			// Check for invalid measurements
			/// @todo Check for no sample value and shadow value once they are non-zero
			if (depth_data[depth_idx] == 0)
			{
				pt.x = pt.y = pt.z = bad_point;
			}
			else
			{
				//test
				//float depth=(-1.0/log(depth_data[depth_idx]))/250.0*9757.0;
				float depth=(-1.0/log(depth_data[depth_idx]))*100;
				
				//Clipping
				if(depth>9757)
					depth=9757;
				if(depth<480)
					depth=0;
				
				//Transform depth buffers
				if(depth_out_view_buffer)
				{
					depth_out_view_buffer[i]=depth/9757.0;
				}
				if(depth_out_raw_buffer)
				{
					depth_out_raw_buffer[i]=depth;
				}
				if(depth_out_float_buffer)
				{
					//TODO: Not Implemented!
				}
				++i;
				
				// Fill in XYZ
				pt.x = (u - pc_center_x) * depth * pc_constant_x;
				pt.y = (v - pc_center_y) * depth * pc_constant_y;
				pt.z = depth * 0.001;
				
				/*
				// Fill in XYZ
				pt.x = (u - pc_center_x) * depth_data[depth_idx] * pc_constant_x;
				pt.y = (v - pc_center_y) * depth_data[depth_idx] * pc_constant_y;
				pt.z = depth_data[depth_idx] * 0.001;
				*/
			}
			
			// Fill in color
			RGBValue color;
			color.Red   = rgb_buffer[color_idx + red_offset];
			color.Green = rgb_buffer[color_idx + green_offset];
			color.Blue  = rgb_buffer[color_idx + blue_offset];
			color.Alpha = 0;
			pt.rgb = color.float_value;
		}
    }
    pointcloud_pub.publish(cloud_msg);
}