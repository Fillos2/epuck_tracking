#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <aruco/aruco.h>

#include <aruco/cvdrawingutils.h>
#include "/usr/local/include/aruco/highlyreliablemarkers.h"

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "epuck_tracking/utils.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <epuck_tracking/bots.h>
using namespace aruco;
using namespace cv;

class EPuckTracking
{
	private:
		cv::Mat inImage, resultImg;
		aruco::CameraParameters camParam;
		bool useRectifiedImages;
		bool draw_markers;
		bool draw_markers_cube;
		bool draw_markers_axis;
		MarkerDetector mDetector;
		vector<Marker> markers;
		BoardConfiguration TheBoardConfig;
		BoardDetector TheBoardDetector;
		Board TheBoardDetected;
		ros::Subscriber cam_info_sub;
		ros::Publisher bots_pub;
		bool cam_info_received;
		image_transport::Publisher image_pub;
		image_transport::Publisher debug_pub;


		std::string board_frame;
		Dictionary D;
		double marker_size;
		std::string board_config;
		float speed[8][2];
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;

		tf::TransformListener _tfListener;

	public:
		EPuckTracking(): cam_info_received(false),nh("~"), it(nh){
			for(int i=0;i<8;i++)for(int j=0;j<2;j++)speed[i][j]=0.0;

			marker_size = 0.05;
			draw_markers=true;
			draw_markers_cube=false;
			draw_markers_axis=true;
			useRectifiedImages=true;
			image_sub = it.subscribe("/camera1/image_rect_color", 1, &EPuckTracking::image_callback, this);
			cam_info_sub = nh.subscribe("/camera1/camera_info", 1, &EPuckTracking::cam_info_callback, this);
			bots_pub = nh.advertise<epuck_tracking::bots>("/epuck_tracking/bots",1000);
			image_pub = it.advertise("result", 1);
			debug_pub = it.advertise("debug", 1);

            if (D.fromFile("/home/filippo/catkin_ws/src/epuck_tracking/data/dizionario.yml") == false)
				ROS_ERROR_STREAM("impossibile aprire dizionario");

            ROS_INFO_STREAM("dictsize "<<D.size());

            if(!HighlyReliableMarkers::loadDictionary(D))
				ROS_INFO("problema con dizionario");
			

            mDetector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
			mDetector.setThresholdParams( 21, 7);
			mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
	        mDetector.setWarpSize((D[0].n()+2)*8);
	        mDetector.setMinMaxSize(0.005, 0.5);

	        TheBoardConfig.readFromFile("/home/filippo/catkin_ws/src/epuck_tracking/data/board3.yml");

	        TheBoardDetector.setYPerperdicular(false);
        	TheBoardDetector.setParams(TheBoardConfig, camParam, marker_size);
        	TheBoardDetector.getMarkerDetector().setThresholdParams(21, 7); // for blue-green markers, the window size has to be larger
        	TheBoardDetector.getMarkerDetector().setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
			TheBoardDetector.getMarkerDetector().setCornerRefinementMethod(aruco::MarkerDetector::LINES);
        	TheBoardDetector.getMarkerDetector().setWarpSize((D[0].n() + 2) * 8);
      		TheBoardDetector.getMarkerDetector().setMinMaxSize(0.005, 0.5);

      		ROS_INFO("setup completo");


			ROS_INFO("Epuck_tracking node started with marker size of %f m and board configuration: %s", marker_size, "board3");

		}

		void image_callback(const sensor_msgs::ImageConstPtr& msg){
			if(!cam_info_received) return;
			static tf::TransformBroadcaster br;
			cv_bridge::CvImagePtr cv_ptr;
			float probDetect = 0.;
			epuck_tracking::bots robots;
			robots.bots.clear();
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inImage = cv_ptr->image;
				resultImg = cv_ptr->image.clone();

				markers.clear();
				mDetector.detect(inImage, markers, camParam, marker_size);


				probDetect=TheBoardDetector.detect(markers, TheBoardConfig,TheBoardDetected, camParam, marker_size);
				tf::Transform boardtf = ar_sys::getTfMarkers(TheBoardDetected.Rvec,TheBoardDetected.Tvec);
				tf::StampedTransform stampedTransform(boardtf, msg->header.stamp, "world", "board");
				br.sendTransform(stampedTransform);


				for (int marker_index = 0; marker_index < markers.size(); marker_index++){
					if(markers[marker_index].id<8){	
					
						tf::Transform transform = ar_sys::getTfMarkers(markers[marker_index].Rvec, markers[marker_index].Tvec);
						
						tf::StampedTransform stampedTransform(transform, msg->header.stamp, "world", boost::to_string(markers[marker_index].id));
						br.sendTransform(stampedTransform);
						tf::Transform lineariz;
						lineariz.setOrigin( tf::Vector3(0.035, 0.0, 0.0) );
						lineariz.setRotation( tf::Quaternion(0, 0, 0, 1) );
						br.sendTransform(tf::StampedTransform(lineariz, msg->header.stamp, boost::to_string(markers[marker_index].id), "linear"+boost::to_string(markers[marker_index].id)));
						robots.bots.push_back(markers[marker_index].id);


					}
					
				}
				bots_pub.publish(robots);
				//for each marker, draw info and its boundaries in the image
				for(size_t i=0; draw_markers && markers[i].id < 8; ++i){
					markers[i].draw(resultImg,cv::Scalar(0,0,255),2);

				}


				if(camParam.isValid() && marker_size != -1){
					//draw a 3d cube in each marker if there is 3d info
					for(size_t i=0; i<markers.size(); ++i){
						if (draw_markers_cube && markers[i].id<8) CvDrawingUtils::draw3dCube(resultImg, markers[i], camParam);
						if (draw_markers_axis==true && markers[i].id<8) CvDrawingUtils::draw3dAxis(resultImg, markers[i], camParam);
					}
					//draw board axis
					if (probDetect > 0.0) CvDrawingUtils::draw3dAxis(resultImg, TheBoardDetected, camParam);
				}

				if(image_pub.getNumSubscribers() > 0){
					//show input with augmented information
					cv_bridge::CvImage out_msg;
					out_msg.header.frame_id = msg->header.frame_id;
					out_msg.header.stamp = msg->header.stamp;
					out_msg.encoding = sensor_msgs::image_encodings::RGB8;
					out_msg.image = resultImg;
					image_pub.publish(out_msg.toImageMsg());
				}

				if(debug_pub.getNumSubscribers() > 0){
					//show also the internal image resulting from the threshold operation
					cv_bridge::CvImage debug_msg;
					debug_msg.header.frame_id = msg->header.frame_id;
					debug_msg.header.stamp = msg->header.stamp;
					debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
					debug_msg.image = mDetector.getThresholdedImage();
					debug_pub.publish(debug_msg.toImageMsg());
				}
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			catch (tf::TransformException& ex){
						ROS_ERROR("ERRORE %s",ex.what());

			}
		}


		// wait for one camerainfo, then shut down that subscriber
		void cam_info_callback(const sensor_msgs::CameraInfo &msg)
		{
			camParam = ar_sys::getCamParams(msg, useRectifiedImages);
			cam_info_received = true;
			cam_info_sub.shutdown();
		}

};


int main(int argc,char **argv)
{
	ros::init(argc, argv, "epuck_tracking");

	EPuckTracking node;

	ros::spin();
}
