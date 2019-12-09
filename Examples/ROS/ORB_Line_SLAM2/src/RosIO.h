#pragma once

// #include <iostream>
// #include <string>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include"../../../include/System.h"

// #include <image_transport/image_transport.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/Image.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>

namespace ORB_SLAM2
{

class RosIO
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	RosIO() {}

	RosIO(ros::NodeHandle& n, ORB_SLAM2::System* ps);
	// void GetPoseFromVIO(list<pair<double, Eigen::Isometry3d>>& lPos_backup);
	bool GetPoseFromVIO(double t, Eigen::Isometry3d& Tvio);

	void SaveTrajectoryTUM(const string &filename)
	{
		if(!Config::useIMU())
			return;
			
		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		for(auto& ele : vtraj)
		{
			double timestamp = ele.tstamp;
			double tx = ele.pos.translation()(0), ty =  ele.pos.translation()(1), tz = ele.pos.translation()(2);

			Eigen::Matrix4d R = ele.pos.matrix();
			vector<float> q = Converter::toQuaternion(Converter::toCvMat(R));

			f << setprecision(6) << timestamp << " " <<  setprecision(9) << tx << " " << ty << " " << tz
			  << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
		}
		f.close();
		cout << endl << "VIO trajectory saved!" << endl;
	}


private:
	// Ros node handle
	ros::NodeHandle nh;
	// ptr ORB_SLAM2::System
	ORB_SLAM2::System* psys;

	// Subscribers and publishers.
	ros::Publisher feature_pub;
	ros::Subscriber pose_sub;

	list<pair<double, Eigen::Isometry3d>> lPos;
	Eigen::Isometry3d Tic;
	Eigen::Isometry3d Twi0;
	bool iscalib;			// to catch the first vio_pose to calibrate pose

	std::mutex mMutexPos;
	std::condition_variable mCond;

	void PoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

	struct Traj {
		Traj(double t, Eigen::Isometry3d p)
			: tstamp(t), pos(p) {}
		double tstamp;
		Eigen::Isometry3d pos;
	};
	vector<Traj> vtraj;
};

RosIO::RosIO(ros::NodeHandle& n, ORB_SLAM2::System* ps) : iscalib(false)
{
	nh = n;
	psys = ps;
	psys->setRosIO(this, bind(&RosIO::GetPoseFromVIO, this, _1, _2));
	pose_sub = nh.subscribe("/firefly_sbx/vio/pose", 50, &ORB_SLAM2::RosIO::PoseCallback, this);

    Eigen::Isometry3d Tc0i0 = Eigen::Isometry3d::Identity();
	Tc0i0.linear()(0, 0)   = 0.014865542981794;
	Tc0i0.linear()(0, 1)   = 0.999557249008346;
	Tc0i0.linear()(0, 2)   =-0.025774436697440;
	Tc0i0.linear()(1, 0)   =-0.999880929698575;
	Tc0i0.linear()(1, 1)   = 0.014967213324719;
	Tc0i0.linear()(1, 2)   = 0.003756188357967;
	Tc0i0.linear()(2, 0)   = 0.004140296794224;
	Tc0i0.linear()(2, 1)   = 0.025715529947966;
	Tc0i0.linear()(2, 2)   = 0.999660727177902;
	Tc0i0.translation()(0) = 0.065222909535531;
	Tc0i0.translation()(1) =-0.020706385492719;
	Tc0i0.translation()(2) =-0.008054602460030;
	Eigen::Isometry3d Twc0 = Eigen::Isometry3d::Identity();
	Twc0.linear()(0, 0)   = 1;
	Twc0.linear()(0, 1)   = 0;
	Twc0.linear()(0, 2)   = 0;
	Twc0.linear()(1, 0)   = 0;
	Twc0.linear()(1, 1)   = 0;
	Twc0.linear()(1, 2)   =-1;
	Twc0.linear()(2, 0)   = 0;
	Twc0.linear()(2, 1)   = 1;
	Twc0.linear()(2, 2)   = 0;
	Twc0.translation()(0) = 0;
	Twc0.translation()(1) = 0;
	Twc0.translation()(2) = 0;
    Twi0 = Twc0 * Tc0i0;
    Tic = Tc0i0.inverse();
}

void RosIO::PoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	Eigen::Isometry3d pos;
	tf::poseMsgToEigen(msg->pose, pos);
	auto tstamp = msg->header.stamp.toSec();

	{
		std::unique_lock<std::mutex> lock(mMutexPos);
		// printf("Insert time %.3f, system time %.3f\n", tstamp, std::chrono::system_clock::now().time_since_epoch().count()/1e9);
		if(!iscalib) {
			Eigen::Isometry3d Tdelta = Twi0 * pos * Tic;
			Twi0 = Tdelta.inverse() * Twi0; // update Twi0 with Tdelta
			iscalib = true;
		}
		lPos.push_back(pair<double, Eigen::Isometry3d>(tstamp, pos));
		mCond.notify_one();
	}

	{
		vtraj.push_back({tstamp, Twi0*pos*Tic});
	}

	return;
}

bool RosIO::GetPoseFromVIO(double t, Eigen::Isometry3d& Tvio_wc)
{
	// printf("Query time %.3f, system time %.3f\n", t, std::chrono::system_clock::now().time_since_epoch().count()/1e9);
	std::unique_lock<std::mutex> lock(mMutexPos);
	mCond.wait_for(lock, std::chrono::milliseconds(20), [this, t]() {return !lPos.empty() && ((t-lPos.back().first)<1e-4);});

	if( !lPos.empty() && ((t-lPos.back().first)<1e-4) ) {
		auto it = lPos.begin();
		while(it!=lPos.end() && t - it->first > 1e-4)
			it = lPos.erase(it);
		assert(it!=lPos.end());
		// printf("delta %f, query time %.3f, take time %.3f, system time %.3f\n", (t-it->first), t,
		// 	it->first, std::chrono::system_clock::now().time_since_epoch().count()/1e9);
		Tvio_wc = Twi0 * it->second * Tic;  //Tdelta * Twc0 * Tc0i0 * Twi * Tic;
		it = lPos.erase(it);
		return true;
	}
	Tvio_wc = Eigen::Isometry3d::Identity(); //Twi0 * Tic;
	return false;
}

}