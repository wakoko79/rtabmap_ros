/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBER_H_
#define INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBER_H_

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>

#include <rtabmap_msgs/RGBDImage.h>
#include <rtabmap_msgs/RGBDImages.h>
#include <rtabmap_msgs/UserData.h>
#include <rtabmap_msgs/OdomInfo.h>
#include <rtabmap_msgs/ScanDescriptor.h>
#include <rtabmap_msgs/SensorData.h>
#include <rtabmap_sync/CommonDataSubscriberDefines.h>
#include <rtabmap_sync/CommonDataSubscriberDefinesThreaded.h>
#include <rtabmap_sync/SyncDiagnostic.h>

#include <boost/thread.hpp>

namespace rtabmap_sync {

class CommonDataSubscriber {
public:
	CommonDataSubscriber(bool gui);
	virtual ~CommonDataSubscriber();

	bool isSubscribedToDepth() const  {return subscribedToDepth_;}
	bool isSubscribedToStereo() const {return subscribedToStereo_;}
	bool isSubscribedToRGB() const  {return subscribedToRGB_;}
	bool isSubscribedToOdom() const  {return subscribedToOdom_;}
	bool isSubscribedToRGBD() const   {return subscribedToRGBD_;}
	bool isSubscribedToScan2d() const {return subscribedToScan2d_;}
	bool isSubscribedToScan3d() const {return subscribedToScan3d_;}
	bool isSubscribedToSensorData() const {return subscribedToSensorData_;}
	bool isSubscribedToOdomInfo() const {return subscribedToOdomInfo_;}
	bool isDataSubscribed() const {return isSubscribedToDepth() || isSubscribedToStereo() || isSubscribedToRGBD() || isSubscribedToScan2d() || isSubscribedToScan3d() || isSubscribedToRGB() || isSubscribedToOdom() || isSubscribedToSensorData();}
	int rgbdCameras() const {return isSubscribedToRGBD()?(int)rgbdSubs_.size():0;}
	int getTopicQueueSize() const {return topicQueueSize_;}
	int getSyncQueueSize() const {return syncQueueSize_;}
	bool isApproxSync() const {return approxSync_;}
	const std::string & name() const {return name_;}

protected:
	void setupCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			const std::string & name,
			std::vector<diagnostic_updater::DiagnosticTask*> otherTasks = std::vector<diagnostic_updater::DiagnosticTask*>());
	virtual void commonMultiCameraCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_msgs::UserDataConstPtr & userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
				const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
				const std::vector<sensor_msgs::CameraInfo> & depthCameraInfoMsgs,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_msgs::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_msgs::GlobalDescriptor>(),
				const std::vector<std::vector<rtabmap_msgs::KeyPoint> > & localKeyPoints = std::vector<std::vector<rtabmap_msgs::KeyPoint> >(),
				const std::vector<std::vector<rtabmap_msgs::Point3f> > & localPoints3d = std::vector<std::vector<rtabmap_msgs::Point3f> >(),
				const std::vector<cv::Mat> & localDescriptors = std::vector<cv::Mat>()) = 0;
	virtual void commonLaserScanCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_msgs::UserDataConstPtr & userDataMsg,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg,
				const rtabmap_msgs::GlobalDescriptor & globalDescriptor = rtabmap_msgs::GlobalDescriptor()) = 0;
	virtual void commonOdomCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_msgs::UserDataConstPtr & userDataMsg,
				const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg) = 0;
	virtual void commonSensorDataCallback(
				const rtabmap_msgs::SensorDataConstPtr & sensorDataMsg,
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg) = 0;

	void commonSingleCameraCallback(
				const nav_msgs::OdometryConstPtr & odomMsg,
				const rtabmap_msgs::UserDataConstPtr & userDataMsg,
				const cv_bridge::CvImageConstPtr & imageMsg,
				const cv_bridge::CvImageConstPtr & depthMsg,
				const sensor_msgs::CameraInfo & rgbCameraInfoMsg,
				const sensor_msgs::CameraInfo & depthCameraInfoMsg,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_msgs::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_msgs::GlobalDescriptor> & globalDescriptorMsgs = std::vector<rtabmap_msgs::GlobalDescriptor>(),
				const std::vector<rtabmap_msgs::KeyPoint> & localKeyPoints = std::vector<rtabmap_msgs::KeyPoint>(),
				const std::vector<rtabmap_msgs::Point3f> & localPoints3d = std::vector<rtabmap_msgs::Point3f>(),
				const cv::Mat & localDescriptors = cv::Mat());

	void tick(const ros::Time & stamp, double targetFrequency = 0);


private:
	void setupDepthCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupStereoCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeOdomInfo);
	void setupRGBCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupRGBDCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupRGBDXCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
#ifdef RTABMAP_SYNC_MULTI_RGBD
	void setupRGBD2Callbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupRGBD3Callbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupRGBD4Callbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo); 
	void setupRGBD5Callbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
	void setupRGBD6Callbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeScan2d,
			bool subscribeScan3d,
			bool subscribeScanDesc,
			bool subscribeOdomInfo);
#endif
    void setupSensorDataCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeOdom,
			bool subscribeOdomInfo);
	void setupScanCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeScan2d,
			bool subscribeScanDesc,
			bool subscribeOdom,
			bool subscribeUserData,
			bool subscribeOdomInfo);
	void setupOdomCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			bool subscribeUserData,
			bool subscribeOdomInfo);

protected:
	std::string subscribedTopicsMsg_;
	int topicQueueSize_;
	int syncQueueSize_;

private:
	bool approxSync_;
	bool subscribedToDepth_;
	bool subscribedToStereo_;
	bool subscribedToRGB_;
	bool subscribedToOdom_;
	bool subscribedToRGBD_;
	bool subscribedToSensorData_;
	bool subscribedToScan2d_;
	bool subscribedToScan3d_;
	bool subscribedToScanDescriptor_;
	bool subscribedToOdomInfo_;
	std::string name_;

	//for depth and rgb-only callbacks
	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

	//for rgbd callback
	ros::Subscriber rgbdSub_;
	std::vector<message_filters::Subscriber<rtabmap_msgs::RGBDImage>*> rgbdSubs_;
	ros::Subscriber rgbdXSubOnly_;
	message_filters::Subscriber<rtabmap_msgs::RGBDImages> rgbdXSub_;

	//for sensor data callback
	ros::Subscriber sensorDataSubOnly_;
	message_filters::Subscriber<rtabmap_msgs::SensorData> sensorDataSub_;

	//stereo callback
	image_transport::SubscriberFilter imageRectLeft_;
	image_transport::SubscriberFilter imageRectRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

	message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
	message_filters::Subscriber<rtabmap_msgs::UserData> userDataSub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> scanSub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> scan3dSub_;
	message_filters::Subscriber<rtabmap_msgs::ScanDescriptor> scanDescSub_;
	message_filters::Subscriber<rtabmap_msgs::OdomInfo> odomInfoSub_;

	ros::Subscriber scan2dSubOnly_;
	ros::Subscriber scan3dSubOnly_;
	ros::Subscriber scanDescSubOnly_;
	ros::Subscriber odomSubOnly_;

	std::unique_ptr<SyncDiagnostic> syncDiagnostic_;

	DECLARE_THREADING_VARIABLES();
	// RGB + Depth
	DATA_SYNCS3_THREADED(depth, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS4_THREADED(depthScan2d, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS4_THREADED(depthScan3d, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS4_THREADED(depthScanDesc, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS4_THREADED(depthInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::OdomInfo);
	DATA_SYNCS5_THREADED(depthScan2dInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS5_THREADED(depthScan3dInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS5_THREADED(depthScanDescInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);

	// RGB + Depth + Odom
	DATA_SYNCS4_THREADED(depthOdom, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS5_THREADED(depthOdomScan2d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS5_THREADED(depthOdomScan3d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS5_THREADED(depthOdomScanDesc, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS5_THREADED(depthOdomInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::OdomInfo);
	DATA_SYNCS6_THREADED(depthOdomScan2dInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS6_THREADED(depthOdomScan3dInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS6_THREADED(depthOdomScanDescInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// RGB + Depth + User Data
	DATA_SYNCS4_THREADED(depthData, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS5_THREADED(depthDataScan2d, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS5_THREADED(depthDataScan3d, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS5_THREADED(depthDataScanDesc, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS5_THREADED(depthDataInfo, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::OdomInfo);
	DATA_SYNCS6_THREADED(depthDataScan2dInfo, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS6_THREADED(depthDataScan3dInfo, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS6_THREADED(depthDataScanDescInfo, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);

	// RGB + Depth + Odom + User Data
	DATA_SYNCS5_THREADED(depthOdomData, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS6_THREADED(depthOdomDataScan2d, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS6_THREADED(depthOdomDataScan3d, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS6_THREADED(depthOdomDataScanDesc, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS6_THREADED(depthOdomDataInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::OdomInfo);
	DATA_SYNCS7_THREADED(depthOdomDataScan2dInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS7_THREADED(depthOdomDataScan3dInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS7_THREADED(depthOdomDataScanDescInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);
#endif

	// Stereo
	DATA_SYNCS4_THREADED(stereo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo);
	DATA_SYNCS5_THREADED(stereoInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, rtabmap_msgs::OdomInfo);

	// Stereo + Odom
	DATA_SYNCS5_THREADED(stereoOdom, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo);
	DATA_SYNCS6_THREADED(stereoOdomInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, rtabmap_msgs::OdomInfo);

	// RGB-only
	DATA_SYNCS2_THREADED(rgb, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS3_THREADED(rgbScan2d, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS3_THREADED(rgbScan3d, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS3_THREADED(rgbScanDesc, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS3_THREADED(rgbInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::OdomInfo);
	DATA_SYNCS4_THREADED(rgbScan2dInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS4_THREADED(rgbScan3dInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS4_THREADED(rgbScanDescInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);

	// RGB-only + Odom
	DATA_SYNCS3_THREADED(rgbOdom, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS4_THREADED(rgbOdomScan2d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS4_THREADED(rgbOdomScan3d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS4_THREADED(rgbOdomScanDesc, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS4_THREADED(rgbOdomInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::OdomInfo);
	DATA_SYNCS5_THREADED(rgbOdomScan2dInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS5_THREADED(rgbOdomScan3dInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS5_THREADED(rgbOdomScanDescInfo, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// RGB-only + User Data
	DATA_SYNCS3_THREADED(rgbData, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS4_THREADED(rgbDataScan2d, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS4_THREADED(rgbDataScan3d, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS4_THREADED(rgbDataScanDesc, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS4_THREADED(rgbDataInfo, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::OdomInfo);
	DATA_SYNCS5_THREADED(rgbDataScan2dInfo, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS5_THREADED(rgbDataScan3dInfo, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS5_THREADED(rgbDataScanDescInfo, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);

	// RGB-only + Odom + User Data
	DATA_SYNCS4_THREADED(rgbOdomData, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS5_THREADED(rgbOdomDataScan2d, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS5_THREADED(rgbOdomDataScan3d, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
	DATA_SYNCS5_THREADED(rgbOdomDataScanDesc, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS5_THREADED(rgbOdomDataInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::OdomInfo);
	DATA_SYNCS6_THREADED(rgbOdomDataScan2dInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS6_THREADED(rgbOdomDataScan3dInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS6_THREADED(rgbOdomDataScanDescInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::Image, sensor_msgs::CameraInfo, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);
#endif

	// 1 RGBD
	void rgbdCallback(const rtabmap_msgs::RGBDImageConstPtr&);
	DATA_SYNCS2_THREADED(rgbdScan2d, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS2_THREADED(rgbdScan3d, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2)
	DATA_SYNCS2_THREADED(rgbdScanDesc, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS2_THREADED(rgbdInfo, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

	// 1 RGBD + Odom
	DATA_SYNCS2_THREADED(rgbdOdom, nav_msgs::Odometry, rtabmap_msgs::RGBDImage);
	DATA_SYNCS3_THREADED(rgbdOdomScan2d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS3_THREADED(rgbdOdomScan3d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS3_THREADED(rgbdOdomScanDesc, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS3_THREADED(rgbdOdomInfo, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 1 RGBD + User Data
	DATA_SYNCS2_THREADED(rgbdData, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage);
	DATA_SYNCS3_THREADED(rgbdDataScan2d, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS3_THREADED(rgbdDataScan3d, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS3_THREADED(rgbdDataScanDesc, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS3_THREADED(rgbdDataInfo, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

	// 1 RGBD + Odom + User Data
	DATA_SYNCS3_THREADED(rgbdOdomData, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage);
	DATA_SYNCS4_THREADED(rgbdOdomDataScan2d, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS4_THREADED(rgbdOdomDataScan3d, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS4_THREADED(rgbdOdomDataScanDesc, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS4_THREADED(rgbdOdomDataInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);
#endif

	// X RGBD
	void rgbdXCallback(const rtabmap_msgs::RGBDImagesConstPtr&);
	DATA_SYNCS2_THREADED(rgbdXScan2d, rtabmap_msgs::RGBDImages, sensor_msgs::LaserScan);
	DATA_SYNCS2_THREADED(rgbdXScan3d, rtabmap_msgs::RGBDImages, sensor_msgs::PointCloud2)
	DATA_SYNCS2_THREADED(rgbdXScanDesc, rtabmap_msgs::RGBDImages, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS2_THREADED(rgbdXInfo, rtabmap_msgs::RGBDImages, rtabmap_msgs::OdomInfo);

	// X RGBD + Odom
	DATA_SYNCS2_THREADED(rgbdXOdom, nav_msgs::Odometry, rtabmap_msgs::RGBDImages);
	DATA_SYNCS3_THREADED(rgbdXOdomScan2d, nav_msgs::Odometry, rtabmap_msgs::RGBDImages, sensor_msgs::LaserScan);
	DATA_SYNCS3_THREADED(rgbdXOdomScan3d, nav_msgs::Odometry, rtabmap_msgs::RGBDImages, sensor_msgs::PointCloud2);
	DATA_SYNCS3_THREADED(rgbdXOdomScanDesc, nav_msgs::Odometry, rtabmap_msgs::RGBDImages, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS3_THREADED(rgbdXOdomInfo, nav_msgs::Odometry, rtabmap_msgs::RGBDImages, rtabmap_msgs::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// X RGBD + User Data
	DATA_SYNCS2_THREADED(rgbdXData, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImages);
	DATA_SYNCS3_THREADED(rgbdXDataScan2d, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImages, sensor_msgs::LaserScan);
	DATA_SYNCS3_THREADED(rgbdXDataScan3d, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImages, sensor_msgs::PointCloud2);
	DATA_SYNCS3_THREADED(rgbdXDataScanDesc, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImages, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS3_THREADED(rgbdXDataInfo, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImages, rtabmap_msgs::OdomInfo);

	// X RGBD + Odom + User Data
	DATA_SYNCS3_THREADED(rgbdXOdomData, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImages);
	DATA_SYNCS4_THREADED(rgbdXOdomDataScan2d, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImages, sensor_msgs::LaserScan);
	DATA_SYNCS4_THREADED(rgbdXOdomDataScan3d, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImages, sensor_msgs::PointCloud2);
	DATA_SYNCS4_THREADED(rgbdXOdomDataScanDesc, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImages, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS4_THREADED(rgbdXOdomDataInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImages, rtabmap_msgs::OdomInfo);
#endif

    // SensorData
	void sensorDataCallback(const rtabmap_msgs::SensorDataConstPtr&);
	DATA_SYNCS2_THREADED(sensorDataInfo, rtabmap_msgs::SensorData, rtabmap_msgs::OdomInfo);

	// SensorData + Odom
	DATA_SYNCS2_THREADED(sensorDataOdom, nav_msgs::Odometry, rtabmap_msgs::SensorData);
	DATA_SYNCS3_THREADED(sensorDataOdomInfo, nav_msgs::Odometry, rtabmap_msgs::SensorData, rtabmap_msgs::OdomInfo);

#ifdef RTABMAP_SYNC_MULTI_RGBD
	// 2 RGBD
	DATA_SYNCS2_THREADED(rgbd2, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS3_THREADED(rgbd2Scan2d, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS3_THREADED(rgbd2Scan3d, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS3_THREADED(rgbd2ScanDesc, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS3_THREADED(rgbd2Info, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

	// 2 RGBD + Odom
	DATA_SYNCS3_THREADED(rgbd2Odom, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS4_THREADED(rgbd2OdomScan2d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS4_THREADED(rgbd2OdomScan3d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS4_THREADED(rgbd2OdomScanDesc, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS4_THREADED(rgbd2OdomInfo, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 2 RGBD + User Data
	DATA_SYNCS3_THREADED(rgbd2Data, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS4_THREADED(rgbd2DataScan2d, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS4_THREADED(rgbd2DataScan3d, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS4_THREADED(rgbd2DataScanDesc, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS4_THREADED(rgbd2DataInfo, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

	// 2 RGBD + Odom + User Data
	DATA_SYNCS4_THREADED(rgbd2OdomData, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS5_THREADED(rgbd2OdomDataScan2d, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS5_THREADED(rgbd2OdomDataScan3d, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS5_THREADED(rgbd2OdomDataScanDesc, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS5_THREADED(rgbd2OdomDataInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);
#endif

	// 3 RGBD
	DATA_SYNCS3_THREADED(rgbd3, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS4_THREADED(rgbd3Scan2d, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS4_THREADED(rgbd3Scan3d, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS4_THREADED(rgbd3ScanDesc, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS4_THREADED(rgbd3Info, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

	// 3 RGBD + Odom
	DATA_SYNCS4_THREADED(rgbd3Odom, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS5_THREADED(rgbd3OdomScan2d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS5_THREADED(rgbd3OdomScan3d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS5_THREADED(rgbd3OdomScanDesc, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS5_THREADED(rgbd3OdomInfo, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 3 RGBD + User Data
	DATA_SYNCS4_THREADED(rgbd3Data, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS5_THREADED(rgbd3DataScan2d, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS5_THREADED(rgbd3DataScan3d, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS5_THREADED(rgbd3DataScanDesc, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS5_THREADED(rgbd3DataInfo, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

	// 3 RGBD + Odom + User Data
	DATA_SYNCS5_THREADED(rgbd3OdomData, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS6_THREADED(rgbd3OdomDataScan2d, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS6_THREADED(rgbd3OdomDataScan3d, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS6_THREADED(rgbd3OdomDataScanDesc, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS6_THREADED(rgbd3OdomDataInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);
#endif

	// 4 RGBD
	DATA_SYNCS4_THREADED(rgbd4, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS5_THREADED(rgbd4Scan2d, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS5_THREADED(rgbd4Scan3d, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS5_THREADED(rgbd4ScanDesc, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS5_THREADED(rgbd4Info, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

	// 4 RGBD + Odom
	DATA_SYNCS5_THREADED(rgbd4Odom, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS6_THREADED(rgbd4OdomScan2d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS6_THREADED(rgbd4OdomScan3d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS6_THREADED(rgbd4OdomScanDesc, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS6_THREADED(rgbd4OdomInfo, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// 4 RGBD + User Data
	DATA_SYNCS5_THREADED(rgbd4Data, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS6_THREADED(rgbd4DataScan2d, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS6_THREADED(rgbd4DataScan3d, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS6_THREADED(rgbd4DataScanDesc, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS6_THREADED(rgbd4DataInfo, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

	// 4 RGBD + Odom + User Data
	DATA_SYNCS6_THREADED(rgbd4OdomData, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS7_THREADED(rgbd4OdomDataScan2d, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS7_THREADED(rgbd4OdomDataScan3d, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS7_THREADED(rgbd4OdomDataScanDesc, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS7_THREADED(rgbd4OdomDataInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);
#endif

	// 5 RGBD
	DATA_SYNCS5_THREADED(rgbd5, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS6_THREADED(rgbd5Scan2d, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS6_THREADED(rgbd5Scan3d, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS6_THREADED(rgbd5ScanDesc, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS6_THREADED(rgbd5Info, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

	// 5 RGBD + Odom
	DATA_SYNCS6_THREADED(rgbd5Odom, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS7_THREADED(rgbd5OdomScan2d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS7_THREADED(rgbd5OdomScan3d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS7_THREADED(rgbd5OdomScanDesc, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS7_THREADED(rgbd5OdomInfo, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

	// 6 RGBD
	DATA_SYNCS6_THREADED(rgbd6, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS7_THREADED(rgbd6Scan2d, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS7_THREADED(rgbd6Scan3d, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS7_THREADED(rgbd6ScanDesc, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS7_THREADED(rgbd6Info, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

	// 6 RGBD + Odom
	DATA_SYNCS7_THREADED(rgbd6Odom, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage);
	DATA_SYNCS8_THREADED(rgbd6OdomScan2d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::LaserScan);
	DATA_SYNCS8_THREADED(rgbd6OdomScan3d, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, sensor_msgs::PointCloud2);
	DATA_SYNCS8_THREADED(rgbd6OdomScanDesc, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS8_THREADED(rgbd6OdomInfo, nav_msgs::Odometry, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::RGBDImage, rtabmap_msgs::OdomInfo);

#endif //RTABMAP_SYNC_MULTI_RGBD

	// Scan
	void scan2dCallback(const sensor_msgs::LaserScanConstPtr&);
	void scan3dCallback(const sensor_msgs::PointCloud2ConstPtr&);
	void scanDescCallback(const rtabmap_msgs::ScanDescriptorConstPtr&);
	DATA_SYNCS2_THREADED(scan2dInfo, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS2_THREADED(scan3dInfo, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS2_THREADED(scanDescInfo, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);

	// Scan + Odom
	DATA_SYNCS2_THREADED(odomScan2d, nav_msgs::Odometry, sensor_msgs::LaserScan);
	DATA_SYNCS2_THREADED(odomScan3d, nav_msgs::Odometry, sensor_msgs::PointCloud2);
	DATA_SYNCS2_THREADED(odomScanDesc, nav_msgs::Odometry, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS3_THREADED(odomScan2dInfo, nav_msgs::Odometry, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS3_THREADED(odomScan3dInfo, nav_msgs::Odometry, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS3_THREADED(odomScanDescInfo, nav_msgs::Odometry, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// Scan + User Data
	DATA_SYNCS2_THREADED(dataScan2d, rtabmap_msgs::UserData, sensor_msgs::LaserScan);
	DATA_SYNCS2_THREADED(dataScan3d, rtabmap_msgs::UserData, sensor_msgs::PointCloud2);
	DATA_SYNCS2_THREADED(dataScanDesc, rtabmap_msgs::UserData, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS3_THREADED(dataScan2dInfo, rtabmap_msgs::UserData, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS3_THREADED(dataScan3dInfo, rtabmap_msgs::UserData, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS3_THREADED(dataScanDescInfo, rtabmap_msgs::UserData, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);

	// Scan + Odom + User Data
	DATA_SYNCS3_THREADED(odomDataScan2d, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::LaserScan);
	DATA_SYNCS3_THREADED(odomDataScan3d, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::PointCloud2);
	DATA_SYNCS3_THREADED(odomDataScanDesc, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::ScanDescriptor);
	DATA_SYNCS4_THREADED(odomDataScan2dInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::LaserScan, rtabmap_msgs::OdomInfo);
	DATA_SYNCS4_THREADED(odomDataScan3dInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, sensor_msgs::PointCloud2, rtabmap_msgs::OdomInfo);
	DATA_SYNCS4_THREADED(odomDataScanDescInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::ScanDescriptor, rtabmap_msgs::OdomInfo);
#endif

	// Odom
	void odomCallback(const nav_msgs::OdometryConstPtr&);
	DATA_SYNCS2_THREADED(odomInfo, nav_msgs::Odometry, rtabmap_msgs::OdomInfo);

#ifdef RTABMAP_SYNC_USER_DATA
	// Odom + User Data
	DATA_SYNCS2_THREADED(odomData, nav_msgs::Odometry, rtabmap_msgs::UserData);
	DATA_SYNCS3_THREADED(odomDataInfo, nav_msgs::Odometry, rtabmap_msgs::UserData, rtabmap_msgs::OdomInfo);
#endif
};

} /* namespace rtabmap_sync */

#endif /* INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBER_H_ */
