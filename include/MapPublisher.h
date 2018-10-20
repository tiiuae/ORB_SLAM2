/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"

namespace ORB_SLAM2
{

class MapPublisher
{
public:
    MapPublisher(Map* pMap, const string &strSettingPath);

    Map* mpMap;

    void Refresh();
    void PublishMapPoints(const std::vector<MapPoint*> &vpMPs, const std::vector<MapPoint*> &vpRefMPs);
    void PublishKeyFrames(const std::vector<KeyFrame*> &vpKFs);
    void PublishCurrentCamera(const cv::Mat &Tcw);
    void SetCurrentCameraPose(const cv::Mat &Tcw);

private:

    cv::Mat GetCurrentCameraPose();
    bool isCamUpdated();
    void ResetCamFlag();

    rclcpp::Node::SharedPtr nh;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher;

    visualization_msgs::msg::Marker mPoints;
    visualization_msgs::msg::Marker mReferencePoints;
    visualization_msgs::msg::Marker mKeyFrames;
    visualization_msgs::msg::Marker mReferenceKeyFrames;
    visualization_msgs::msg::Marker mCovisibilityGraph;
    visualization_msgs::msg::Marker mMST;
    visualization_msgs::msg::Marker mCurrentCamera;

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;
    bool mbCameraUpdated;

    std::mutex mMutexCamera;
};

} //namespace ORB_SLAM

#endif // MAPPUBLISHER_H
