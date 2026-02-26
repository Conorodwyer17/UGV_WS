#include "server_workers.h"
#include "slamware_ros_sdk_server.h"
#include <cassert>
#include <limits>
#include <random>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace rp::standalone::aurora;
namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////
     ServerOdometryWorker::ServerOdometryWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const std::chrono::milliseconds& triggerInterval
        )
        : super_t(pRosSdkServer, wkName, triggerInterval)
        , lastPoseStamped_(geometry_msgs::msg::PoseStamped())
        , firstPoseReceived_(false)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubOdometry_ = nhRos->create_publisher<nav_msgs::msg::Odometry>(srvParams.getParameter<std::string>(std::string("odom_topic")), 10);
    }

    ServerOdometryWorker::~ServerOdometryWorker()
    {
        //
    }

    bool ServerOdometryWorker::reinitWorkLoop()
    {
        if (!this->super_t::reinitWorkLoop())
            return false;
        firstPoseReceived_ = false;
        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    double ServerOdometryWorker::getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat)
    {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        return tf2::getYaw(q);
    }

    void ServerOdometryWorker::doPerform()
    {
        const auto& srvParams = serverParams();
        auto wkDat = mutableWorkData();
        const auto& currentPoseStamped = wkDat->robotPose;

        double vx = 0.0, vy = 0.0, vth = 0.0;
        if (!firstPoseReceived_) {
            firstPoseReceived_ = true;
            lastPoseStamped_ = currentPoseStamped;
            // Publish first pose immediately so TF odom->base_link exists (mission and Nav2 require it).
            // Use zero twist; subsequent cycles will publish velocity when dt > 0.
        } else {
            double dt = (rclcpp::Time(currentPoseStamped.header.stamp) -
                         rclcpp::Time(lastPoseStamped_.header.stamp)).seconds();
            if (dt < std::numeric_limits<double>::epsilon()) {
                return;
            }
            float deltaX = currentPoseStamped.pose.position.x - lastPoseStamped_.pose.position.x;
            float deltaY = currentPoseStamped.pose.position.y - lastPoseStamped_.pose.position.y;
            double deltaYaw = getYawFromQuaternion(currentPoseStamped.pose.orientation) -
                              getYawFromQuaternion(lastPoseStamped_.pose.orientation);
            vx = deltaX / dt;
            vy = deltaY / dt;
            vth = deltaYaw / dt;
            lastPoseStamped_ = currentPoseStamped;
        }

        // Use node clock for header so TF/odom are always current (device pose stamp can be zero or stale).
        const rclcpp::Time nowStamp = rosSdkServer()->get_clock()->now();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = nowStamp;
        odom.header.frame_id = srvParams.getParameter<std::string>("odom_frame");
        odom.child_frame_id = srvParams.getParameter<std::string>("robot_frame");
        odom.pose.pose = currentPoseStamped.pose;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        pubOdometry_->publish(odom);

        auto &tfBrdcst = tfBroadcaster();
        geometry_msgs::msg::TransformStamped odomTrans;
        odomTrans.header.stamp = nowStamp;
        odomTrans.header.frame_id = srvParams.getParameter<std::string>("odom_frame");
        odomTrans.child_frame_id = srvParams.getParameter<std::string>("robot_frame");
        odomTrans.transform.translation.x = currentPoseStamped.pose.position.x;
        odomTrans.transform.translation.y = currentPoseStamped.pose.position.y;
        odomTrans.transform.translation.z = currentPoseStamped.pose.position.z;
        odomTrans.transform.rotation = currentPoseStamped.pose.orientation;
        tfBrdcst->sendTransform(odomTrans);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerRobotPoseWorker::ServerRobotPoseWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const std::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval),lastTimestamp_(0)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubRobotPose_ = nhRos->create_publisher<geometry_msgs::msg::PoseStamped>(srvParams.getParameter<std::string>(std::string("robot_pose_topic")), 10);
    }

    ServerRobotPoseWorker::~ServerRobotPoseWorker()
    {
        //
    }

    void ServerRobotPoseWorker::doPerform()
    {
        const auto& srvParams = serverParams();
        auto& tfBrdcst = tfBroadcaster();
        auto wkDat = mutableWorkData();
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "doPerform(), auroraSDK is null.");
            return;
        }

        slamtec_aurora_sdk_pose_se3_t pose;
        uint64_t pose_timestamp;
        if (!auroraSDK->dataProvider.getCurrentPoseSE3WithTimestamp(pose, pose_timestamp))
            return;
        if (pose_timestamp < lastTimestamp_)
            return;
        lastTimestamp_ = pose_timestamp;
        wkDat->robotPose.header.stamp = rclcpp::Clock().now();
        wkDat->robotPose.pose.position.x = pose.translation.x;
        wkDat->robotPose.pose.position.y = pose.translation.y;
        wkDat->robotPose.pose.position.z = pose.translation.z;

        wkDat->robotPose.pose.orientation.x = pose.quaternion.x;
        wkDat->robotPose.pose.orientation.y = pose.quaternion.y;
        wkDat->robotPose.pose.orientation.z = pose.quaternion.z;
        wkDat->robotPose.pose.orientation.w = pose.quaternion.w;

        geometry_msgs::msg::PoseStamped ros_pose;
        ros_pose.header.frame_id = srvParams.getParameter<std::string>("map_frame");
        ros_pose.header.stamp = wkDat->robotPose.header.stamp;
        ros_pose.pose = wkDat->robotPose.pose;
        pubRobotPose_->publish(ros_pose);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerExploreMapUpdateWorker::ServerExploreMapUpdateWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const std::chrono::milliseconds& triggerInterval
        )
        : super_t(pRosSdkServer, wkName, triggerInterval)
        , shouldReinitMap_(true)
        , hasSyncedWholeMap_(false)
    {
        //
    }

    ServerExploreMapUpdateWorker::~ServerExploreMapUpdateWorker()
    {
        //
    }

    bool ServerExploreMapUpdateWorker::reinitWorkLoop()
    {
        
        if (!this->super_t::reinitWorkLoop())
            return false;
        isWorkLoopInitOk_ = false;
        hasSyncedWholeMap_ = false;
        requestReinitMap_();
        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerExploreMapUpdateWorker::doPerform()
    {
        const auto& srvParams = serverParams();
        auto wkDat = mutableWorkData();

        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK) {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "Failed to get aurora sdk");
            return;
        }

        if (!checkToReinitMap_(auroraSDK, wkDat))
            return;

        if(!hasSyncedWholeMap_) {
            rclcpp::Duration dt = rclcpp::Clock().now() - mapInitTime_;
            if(dt.seconds() > 15) {
                wkDat->syncMapRequested.store(true);
                hasSyncedWholeMap_ = true;
                RCLCPP_INFO(rclcpp::get_logger("server workers"), "sync the map after preview map generated.");
            }
        }

        if (wkDat->syncMapRequested.load())
        {
            RCLCPP_INFO(rclcpp::get_logger("server workers"), "try to sync whold explore map.");
            if (syncWholeMap_(srvParams, auroraSDK, wkDat))
            {
                wkDat->syncMapRequested.store(false);
                RCLCPP_INFO(rclcpp::get_logger("server workers"), "whold explore map synchronized.");
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("server workers"), "failed to sync whole explore map.");
            }
            return;
        }

        updateMapNearRobot_(srvParams, auroraSDK, wkDat);
    }

    void ServerExploreMapUpdateWorker::requestReinitMap_()
    {
        shouldReinitMap_ = true;
    }

    bool ServerExploreMapUpdateWorker::checkToReinitMap_(rp::standalone::aurora::RemoteSDK *sdk, const ServerWorkData_Ptr &wkDat)
    {
        if (!shouldReinitMap_)
            return true;

        RCLCPP_INFO(rclcpp::get_logger("server workers"), "try to reinit explore map.");
        wkDat->syncMapRequested.store(true);

        sdk->lidar2DMapBuilder.requireRedrawPreviewMap();
        LIDAR2DGridMapGenerationOptions genOption;
        sdk->lidar2DMapBuilder.getPreviewMapGenerationOptions(genOption);
        wkDat->exploreMapHolder.reinit(genOption.resolution);

        RCLCPP_INFO(
            rclcpp::get_logger("server workers"),
            "explore map initialized, resolution: %f, moreCellCntToExtend: %d.",
            genOption.resolution,
            wkDat->exploreMapHolder.getMoreCellCountToExtend());
        shouldReinitMap_ = false;
        mapInitTime_ = rclcpp::Clock().now();
        return true;
    }

    bool ServerExploreMapUpdateWorker::checkRecvResolution_(float recvResolution, const ServerWorkData_Ptr& wkDat)
    {
        const auto fResolution = wkDat->exploreMapHolder.resolution();
        if (std::abs(fResolution - recvResolution) <
            std::numeric_limits<float>::epsilon())
          return true;

        RCLCPP_ERROR(rclcpp::get_logger("server workers"), "local resolution: %f, received resolution: %f, request reinit.", fResolution, recvResolution);
        requestReinitMap_();
        return false;
    }

    bool ServerExploreMapUpdateWorker::updateMapInCellIdxRect_(const rp::standalone::aurora::OccupancyGridMap2DRef& prevMap
            , const utils::RectangleF& reqRect
            , const ServerWorkData_Ptr& wkDat
            )
    {
        const auto& fResolution = prevMap.getResolution();
        
        static const size_t MAX_FETCH_BLOCK_CXCY = 16 * 1024; // 16*16 to get 256 MB
        size_t fetchSize = (size_t)(reqRect.width() * reqRect.height() / fResolution / fResolution);
        if (fetchSize > (MAX_FETCH_BLOCK_CXCY * MAX_FETCH_BLOCK_CXCY)) {
            // using smaller block to fetch
            float fetchLogicCXCY = MAX_FETCH_BLOCK_CXCY * fResolution;
            for (float y = 0; y < reqRect.height(); y += fetchLogicCXCY) {
                for (float x = 0; x < reqRect.width(); x += fetchLogicCXCY) {
                    slamtec_aurora_sdk_rect_t fetchRect;
                    fetchRect.x = x + reqRect.x();
                    fetchRect.y = y + reqRect.y();
                    fetchRect.width = fetchLogicCXCY;
                    fetchRect.height = fetchLogicCXCY;
                    std::vector<uint8_t> mapData;
                    slamtec_aurora_sdk_2d_gridmap_fetch_info_t info;
                    prevMap.readCellData(fetchRect, info, mapData, false);
                    wkDat->exploreMapHolder.setMapData(info.real_x, info.real_y, fResolution, info.cell_width, info.cell_height, mapData.data());
                }
            }
        }
        else {
            slamtec_aurora_sdk_rect_t fetchRect;
            fetchRect.x = reqRect.x();
            fetchRect.y = reqRect.y();
            fetchRect.width = reqRect.width();
            fetchRect.height = reqRect.height();
            std::vector<uint8_t> mapData;
            slamtec_aurora_sdk_2d_gridmap_fetch_info_t info;
            prevMap.readCellData(fetchRect, info, mapData, false);
            wkDat->exploreMapHolder.setMapData(info.real_x, info.real_y, fResolution, info.cell_width, info.cell_height, mapData.data());
        }
        
        if (!checkRecvResolution_(fResolution, wkDat))
            return false;

        return true;
    }

    bool ServerExploreMapUpdateWorker::syncWholeMap_(const ServerParams& srvParams
        , rp::standalone::aurora::RemoteSDK *sdk
        , const ServerWorkData_Ptr& wkDat
        )
    {
        auto &prevMap = sdk->lidar2DMapBuilder.getPreviewMap();

        slamtec_aurora_sdk_2dmap_dimension_t mapDimension;
        prevMap.getMapDimension(mapDimension);
        utils::RectangleF knownArea(mapDimension.min_x, mapDimension.min_y,
            mapDimension.max_x - mapDimension.min_x,
            mapDimension.max_y - mapDimension.min_y);

        const auto knownCellIdxRect = wkDat->exploreMapHolder.calcRoundedCellIdxRect(knownArea);
        RCLCPP_INFO(rclcpp::get_logger("server workers"), "known area: ((%f, %f), (%f, %f)), cell rect: ((%d, %d), (%d, %d))"
            , knownArea.x(), knownArea.y(), knownArea.width(), knownArea.height()
            , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
            );
        //Maintain consistency between ROS1 and ROS2 code to eliminate unnecessary errors
        // if (ServerMapHolder::sfIsCellIdxRectEmpty(knownCellIdxRect) || 
        //     knownCellIdxRect.width() < MIN_VALID_MAP_DIMENSION || knownCellIdxRect.height() < MIN_VALID_MAP_DIMENSION)
        if (ServerMapHolder::sfIsCellIdxRectEmpty(knownCellIdxRect))
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "sync map, knownCellIdxRect is empty or too small.");
            return false;
        }

        wkDat->exploreMapHolder.clear();
        wkDat->exploreMapHolder.reserveByCellIdxRect(knownCellIdxRect);

        return updateMapInCellIdxRect_(prevMap, knownArea, wkDat);
    }

    bool ServerExploreMapUpdateWorker::updateMapNearRobot_(const ServerParams& srvParams
            , rp::standalone::aurora::RemoteSDK *sdk
            , const ServerWorkData_Ptr& wkDat
            )
    {
        float fVal(2.0f);
        srvParams.get_parameter<float>("map_update_near_robot_half_wh", fVal);
        const float fHalfWH = std::max<float>(2.0f, fVal);
        const float fWH = fHalfWH + fHalfWH;
        const auto nearRobotArea = utils::RectangleF(
            static_cast<float>(wkDat->robotPose.pose.position.x - fHalfWH),
            static_cast<float>(wkDat->robotPose.pose.position.y - fHalfWH),
            fWH, fWH);
        const auto nearRobotIdxRect = wkDat->exploreMapHolder.calcMinBoundingCellIdxRect(nearRobotArea);

        auto &prevMap = sdk->lidar2DMapBuilder.getPreviewMap();
        slamtec_aurora_sdk_2dmap_dimension_t mapDimension;
        prevMap.getMapDimension(mapDimension);
        utils::RectangleF knownArea(mapDimension.min_x, mapDimension.min_y,
            mapDimension.max_x - mapDimension.min_x,
            mapDimension.max_y - mapDimension.min_y);
        const auto knownCellIdxRect = wkDat->exploreMapHolder.calcRoundedCellIdxRect(knownArea);
    #if 0
        RCLCPP_INFO(rclcpp::get_logger("server workers"), "known area: ((%f, %f), (%f, %f)), cell rect: ((%d, %d), (%d, %d))."
            , knownArea.x(), knownArea.y(), knownArea.width(), knownArea.height()
            , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
            );
    #endif
        if (ServerMapHolder::sfIsCellIdxRectEmpty(knownCellIdxRect))
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "update map, knownCellIdxRect is empty, request sync map.");
            rosSdkServer()->requestSyncMap();
            return false;
        }

        const auto reqIdxRect = ServerMapHolder::sfIntersectionOfCellIdxRect(nearRobotIdxRect, knownCellIdxRect);
        if (ServerMapHolder::sfIsCellIdxRectEmpty(reqIdxRect))
        {
            RCLCPP_WARN(rclcpp::get_logger("server workers"), "knownCellIdxRect: ((%d, %d), (%d, %d)), nearRobotIdxRect: ((%d, %d), (%d, %d)), intersection is empty."
                , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
                , nearRobotIdxRect.x(), nearRobotIdxRect.y(), nearRobotIdxRect.width(), nearRobotIdxRect.height()
                );
            return false;
        }
        const auto& reqArea = wkDat->exploreMapHolder.calcAreaByCellIdxRect(reqIdxRect);

        return updateMapInCellIdxRect_(prevMap, reqArea, wkDat);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerExploreMapPublishWorker::ServerExploreMapPublishWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const std::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        // Use TRANSIENT_LOCAL QoS so Nav2 costmap (default map_subscribe_transient_local: True) can receive the map.
        // Aurora previously used VOLATILE (default), causing DURABILITY_QOS_POLICY mismatch with Nav2.
        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.transient_local();
        map_qos.reliable();
        pubMapDat_ = nhRos->create_publisher<nav_msgs::msg::OccupancyGrid>(srvParams.getParameter<std::string>("map_topic"), map_qos);
        pubMapInfo_ = nhRos->create_publisher<nav_msgs::msg::MapMetaData>(srvParams.getParameter<std::string>("map_info_topic"), 1);
    }

    ServerExploreMapPublishWorker::~ServerExploreMapPublishWorker()
    {
        //
    }

    void ServerExploreMapPublishWorker::doPerform()
    {
        const auto& srvParams = serverParams();
        auto wkDat = workData();

        if (wkDat->exploreMapHolder.isMapDataEmpty())
        {
            //RCLCPP_WARN(rclcpp::get_logger("server workers"), "current explore map data is empty.");
            return;
        }

        nav_msgs::srv::GetMap::Response msgMap;
        wkDat->exploreMapHolder.fillRosMapMsg(msgMap);

        // Set the header information on the map
        msgMap.map.header.stamp = rclcpp::Clock().now();
        msgMap.map.header.frame_id = srvParams.getParameter<std::string>("map_frame"); //srvParams.map_frame;

        pubMapDat_->publish(msgMap.map);
        pubMapInfo_->publish(msgMap.map.info);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerLaserScanWorker::ServerLaserScanWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const std::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
        , compensatedAngleCnt_(360u)
        , absAngleIncrement_(C_FLT_2PI / compensatedAngleCnt_)
        , latestLidarStartTimestamp_(0)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubLaserScan_ = nhRos->create_publisher<sensor_msgs::msg::LaserScan>(srvParams.getParameter<std::string>("scan_topic"), 10); // srvParams.scan_topic, 10);
    }

    ServerLaserScanWorker::~ServerLaserScanWorker()
    {
        //
    }

    void ServerLaserScanWorker::doPerform()
    {
        auto aurora = rosSdkServer()->safeGetAuroraSdk();
        if(!aurora)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get aurora sdk");
            return;
        }
        const auto& srvParams = serverParams();
        auto& tfBrdcst = tfBroadcaster();
        auto wkDat = workData();

        rclcpp::Time startScanTime(0), endScanTime(0);
        rp::standalone::aurora::SingleLayerLIDARScan scan;
        slamtec_aurora_sdk_pose_se3_t poseSE3;
        startScanTime = rclcpp::Clock().now();
        if(!aurora->dataProvider.peekRecentLIDARScanSingleLayer(scan, poseSE3, false)) {
            return;
        }
        endScanTime = rclcpp::Clock().now();
        double dblScanDur = (endScanTime - startScanTime).seconds();

        const auto& points = scan.scanData;
        if (points.size() < 2)
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "laser points count: %u, too small, skip publish.", (unsigned int)points.size());
            return;
        }

        sensor_msgs::msg::LaserScan msgScan;
        msgScan.header.stamp = startScanTime;
        msgScan.header.frame_id = srvParams.getParameter<std::string>("laser_frame"); //srvParams.laser_frame;
        fillRangeMinMaxInMsg_(points, msgScan);

        bool isLaserDataReverse = false;
        if ((points.back().angle < points.front().angle && !srvParams.getParameter<bool>("ladar_data_clockwise")) ||
            (points.back().angle > points.front().angle && srvParams.getParameter<bool>("ladar_data_clockwise")))
        {
            isLaserDataReverse = true;
        }

        // if (srvParams.angle_compensate)
        if (srvParams.getParameter<bool>("angle_compensate"))
        {
            compensateAndfillRangesInMsg_(points, srvParams.getParameter<bool>("ladar_data_clockwise"), isLaserDataReverse, msgScan);
        }
        else
        {
            fillOriginalRangesInMsg_(points, isLaserDataReverse, msgScan);
        }
        assert(2 <= msgScan.ranges.size());
        msgScan.scan_time = dblScanDur;
        msgScan.time_increment = dblScanDur / (double)(msgScan.ranges.size() - 1);

        pubLaserScan_->publish(msgScan);

        geometry_msgs::msg::TransformStamped LaserTrans;
        LaserTrans.header.stamp = msgScan.header.stamp;
        LaserTrans.header.frame_id = srvParams.getParameter<std::string>("map_frame");
        LaserTrans.child_frame_id = srvParams.getParameter<std::string>("laser_frame");
        LaserTrans.transform.translation.x = poseSE3.translation.x;
        LaserTrans.transform.translation.y = poseSE3.translation.y;
        LaserTrans.transform.translation.z = poseSE3.translation.z;
        LaserTrans.transform.rotation.x = poseSE3.quaternion.x;
        LaserTrans.transform.rotation.y = poseSE3.quaternion.y;
        LaserTrans.transform.rotation.z = poseSE3.quaternion.z;
        LaserTrans.transform.rotation.w = poseSE3.quaternion.w;
        tfBrdcst->sendTransform(LaserTrans);
    }

    void ServerLaserScanWorker::fillRangeMinMaxInMsg_(const std::vector<slamtec_aurora_sdk_lidar_scan_point_t> & laserPoints
            , sensor_msgs::msg::LaserScan& msgScan
            ) const
    {
        msgScan.range_min = std::numeric_limits<float>::infinity();
        msgScan.range_max = 0.0f;
        for (auto cit = laserPoints.cbegin(), citEnd = laserPoints.cend(); citEnd != cit; ++cit)
        {
            const float tmpDist = cit->dist;
            if (tmpDist < msgScan.range_min)
            {
                msgScan.range_min = std::max<float>(0.0f, tmpDist);
            }
            if (msgScan.range_max < tmpDist)
            {
                msgScan.range_max = tmpDist;
            }
        }
    }

    float ServerLaserScanWorker::calcAngleInNegativePiToPi_(float angle) const
    {
        float fRes = std::fmod(angle + C_FLT_PI, C_FLT_2PI);
        if (fRes < 0.0f)
            fRes += C_FLT_2PI;
        fRes -= C_FLT_PI;

        if (fRes < -C_FLT_PI)
            fRes = -C_FLT_PI;
        else if (C_FLT_PI <= fRes)
            fRes = -C_FLT_PI;
        return fRes;
    }

    std::uint32_t ServerLaserScanWorker::calcCompensateDestIndexBySrcAngle_(float srcAngle
        , bool isAnglesReverse
        ) const
    {
        assert(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);
        
        float fDiff = (isAnglesReverse ? (C_FLT_PI - srcAngle) : (srcAngle + C_FLT_PI));
        fDiff = std::max<float>(0.0f, fDiff);
        fDiff = std::min<float>(fDiff, C_FLT_2PI);

        std::uint32_t destIdx = static_cast<std::uint32_t>(std::round(fDiff / absAngleIncrement_));
        if (compensatedAngleCnt_ <= destIdx)
            destIdx = 0;
        return destIdx;
    }

    bool ServerLaserScanWorker::isSrcAngleMoreCloseThanOldSrcAngle_(float srcAngle, float destAngle, float oldSrcAngle) const
    {
        assert(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);
        assert(-C_FLT_PI <= destAngle && destAngle < C_FLT_PI);
        assert(-C_FLT_PI <= oldSrcAngle && oldSrcAngle < C_FLT_PI);

        float newDiff = std::abs(destAngle - srcAngle);
        if (C_FLT_2PI <= newDiff)
            newDiff = 0.0f;
        else if (C_FLT_PI < newDiff)
            newDiff = C_FLT_2PI - newDiff;

        float oldDiff = std::abs(destAngle - oldSrcAngle);
        if (C_FLT_2PI <= oldDiff)
            oldDiff = 0.0f;
        else if (C_FLT_PI < oldDiff)
            oldDiff = C_FLT_2PI - oldDiff;

        return (newDiff < oldDiff);
    }

    void ServerLaserScanWorker::compensateAndfillRangesInMsg_(const std::vector<slamtec_aurora_sdk_lidar_scan_point_t> & laserPoints
            , bool isClockwise
            , bool isLaserDataReverse
            , sensor_msgs::msg::LaserScan& msgScan
            ) const
    {
        assert(2 <= laserPoints.size());

        msgScan.ranges.clear();
        msgScan.intensities.clear();
        msgScan.ranges.resize(compensatedAngleCnt_, std::numeric_limits<float>::infinity());
        msgScan.intensities.resize(compensatedAngleCnt_, 0.0);

        if (!isClockwise)
        {
            msgScan.angle_min = -C_FLT_PI;
            msgScan.angle_max = C_FLT_PI - absAngleIncrement_;
            msgScan.angle_increment = absAngleIncrement_;
        }
        else
        {
            msgScan.angle_min = C_FLT_PI;
            msgScan.angle_max = (-C_FLT_PI + absAngleIncrement_); 
            msgScan.angle_increment = -absAngleIncrement_;
        }

        std::vector<float> tmpSrcAngles(compensatedAngleCnt_);
        if (!isLaserDataReverse)
        {
            for (int i = 0; i < laserPoints.size(); ++i)
            {
                compensateAndfillRangeInMsg_(laserPoints[i], isClockwise, msgScan, tmpSrcAngles);
            }
        }
        else
        {
            for (int i = laserPoints.size() - 1; i >= 0; --i)
            {
                compensateAndfillRangeInMsg_(laserPoints[i], isClockwise, msgScan, tmpSrcAngles);
            }
        }
    }

    void ServerLaserScanWorker::compensateAndfillRangeInMsg_(const slamtec_aurora_sdk_lidar_scan_point_t& laserPoint
            , bool isClockwise
            , sensor_msgs::msg::LaserScan& msgScan
            , std::vector<float>& tmpSrcAngles
            ) const
    {
        const float srcAngle = calcAngleInNegativePiToPi_(laserPoint.angle);
        const std::uint32_t destIdx = calcCompensateDestIndexBySrcAngle_(srcAngle, isClockwise);
        assert(destIdx < compensatedAngleCnt_);
        const float destAngle = calcAngleInNegativePiToPi_(msgScan.angle_min + msgScan.angle_increment * destIdx);

        const bool shouldWrite = (std::isinf(msgScan.ranges[destIdx])
            || isSrcAngleMoreCloseThanOldSrcAngle_(srcAngle, destAngle, tmpSrcAngles[destIdx])
            );
        if (shouldWrite)
        {
            msgScan.ranges[destIdx] = laserPoint.dist;
            msgScan.intensities[destIdx] = laserPoint.quality;
            tmpSrcAngles[destIdx] = srcAngle;
        }
    }

    void ServerLaserScanWorker::fillOriginalRangesInMsg_(const std::vector<slamtec_aurora_sdk_lidar_scan_point_t>& laserPoints
            , bool isLaserDataReverse
            , sensor_msgs::msg::LaserScan& msgScan
            ) const
    {
        msgScan.intensities.resize(laserPoints.size());
        msgScan.ranges.resize(laserPoints.size());

        int index = 0;
        if (!isLaserDataReverse)
        {
            for (int i = 0; i < laserPoints.size(); ++i)
            {
                fillOriginalRangeInMsg_(laserPoints[i], index++, msgScan);
            }
            msgScan.angle_min =  laserPoints.front().angle;
            msgScan.angle_max =  laserPoints.back().angle;
            msgScan.angle_increment = (msgScan.angle_max - msgScan.angle_min) / (double)(msgScan.ranges.size() - 1);
        }
        else
        {
            for (int i = laserPoints.size() - 1; i >= 0; --i)
            {
                fillOriginalRangeInMsg_(laserPoints[i], index++, msgScan);
            }
            msgScan.angle_min =  laserPoints.back().angle;
            msgScan.angle_max =  laserPoints.front().angle;
            msgScan.angle_increment = (msgScan.angle_max - msgScan.angle_min) / (double)(msgScan.ranges.size() - 1);
        }
    }

    void ServerLaserScanWorker::fillOriginalRangeInMsg_(const slamtec_aurora_sdk_lidar_scan_point_t& laserPoint
            , int index
            , sensor_msgs::msg::LaserScan& msgScan
            ) const
    {
        msgScan.ranges[index] = laserPoint.dist;
        msgScan.intensities[index] = laserPoint.quality;
    }

    ServerSystemStatusWorker::ServerSystemStatusWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const std::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval), lastDeviceStatus_(std::nullopt),
        lastRelocalizationStatus_(std::nullopt)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubSystemStatus_ = nhRos->create_publisher<slamware_ros_sdk::msg::SystemStatus>(srvParams.getParameter<std::string>("system_status_topic_name"), 1);
        pubRelocalizaitonStatus_ = nhRos->create_publisher<slamware_ros_sdk::msg::RelocalizationStatus>(srvParams.getParameter<std::string>("relocalization_status_topic_name"), 1);
    }

    ServerSystemStatusWorker::~ServerSystemStatusWorker()
    {
        //
    }

    bool ServerSystemStatusWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }

    void ServerSystemStatusWorker::doPerform()
    {
        auto aurora = rosSdkServer()->safeGetAuroraSdk();
        if(!aurora)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get aurora sdk");
            return;
        }

        slamtec_aurora_sdk_device_status_desc_t result;
        slamware_ros_sdk::msg::SystemStatus resp;
        if(aurora->dataProvider.peekVSLAMSystemStatus(result))
        {
            //get current time in ns
            resp.timestamp_ns = rclcpp::Clock().now().nanoseconds();
            switch (result.status)  
            {
            case SLAMTEC_AURORA_SDK_DEVICE_INITED:
            {
                if(lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == SLAMTEC_AURORA_SDK_DEVICE_INITED)
                {
                    resp.status = "DeviceRunning";
                }
                else 
                    resp.status = "DeviceInited";
                break;
            }
            case SLAMTEC_AURORA_SDK_DEVICE_INIT_FAILED:
                resp.status = "DeviceInitFailed";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_LOOP_CLOSURE:
                resp.status = "DeviceLoopClosure";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_OPTIMIZATION_COMPLETED:
            {
                if(lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == SLAMTEC_AURORA_SDK_DEVICE_OPTIMIZATION_COMPLETED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceOptimizationCompleted";
                break;
            }
            case SLAMTEC_AURORA_SDK_DEVICE_TRACKING_LOST:
                resp.status = "DeviceTrackingLost";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_TRACKING_RECOVERED:
            {
                if(lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == SLAMTEC_AURORA_SDK_DEVICE_TRACKING_RECOVERED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceTrackingRecovered";
                break;
            }
            case  SLAMTEC_AURORA_SDK_DEVICE_MAP_CLEARED:    
                resp.status = "DeviceMapCleared";
                break;
            case  SLAMTEC_AURORA_SDK_DEVICE_MAP_LOADING_STARTED:
                resp.status = "DeviceMapLoadingStarted";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_MAP_SAVING_STARTED:
                resp.status = "DeviceMapSavingStarted";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_MAP_LOADING_COMPLETED:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == SLAMTEC_AURORA_SDK_DEVICE_MAP_LOADING_COMPLETED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceMapLoadingCompleted";
                break;
            }
            case SLAMTEC_AURORA_SDK_DEVICE_MAP_SAVING_COMPLETED:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == SLAMTEC_AURORA_SDK_DEVICE_MAP_SAVING_COMPLETED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceMapSavingCompleted";
                break;
            }
            case 13:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == 13)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceRelocSuccess";
                break;
            }
            case 14:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == 14)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceRelocFailed";
                break;
            }
            case 15:
            {
                if (lastDeviceStatus_.has_value() && lastDeviceStatus_.value() == 15)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceRelocCancelled";
                break;
            }
            case 16:
                resp.status = "DeviceRelocRunning";
                break;
            default:
                return;
            }

            lastDeviceStatus_ = result.status;
            pubSystemStatus_->publish(resp);
        }

        slamtec_aurora_sdk_relocalization_status_t reloc_status;
        slamware_ros_sdk::msg::RelocalizationStatus reloc_resp;
        if(aurora->dataProvider.peekRelocalizationStatus(reloc_status))
        {
            reloc_resp.timestamp_ns = rclcpp::Clock().now().nanoseconds();
            {
                switch (reloc_status.status)
                {
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_NONE:
                    reloc_resp.status = "RelocalizationNone";
                    break;
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_STARTED:
                    reloc_resp.status = "RelocalizationRunning";
                    break;
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_SUCCEED:
                {
                    if(lastRelocalizationStatus_.has_value() && lastRelocalizationStatus_.value() == SLAMTEC_AURORA_SDK_RELOCALIZATION_SUCCEED)
                    {
                        reloc_resp.status = "RelocalizationNone";
                    }
                    else
                        reloc_resp.status = "RelocalizationSucceed";
                    break;
                }
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_FAILED:
                {
                    if(lastRelocalizationStatus_.has_value() && lastRelocalizationStatus_.value() == SLAMTEC_AURORA_SDK_RELOCALIZATION_FAILED)
                    {
                        reloc_resp.status = "RelocalizationNone";
                    }
                    else
                        reloc_resp.status = "RelocalizationFailed";
                    break;
                }
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_ABORTED:
                {
                    if(lastRelocalizationStatus_.has_value() && lastRelocalizationStatus_.value() == SLAMTEC_AURORA_SDK_RELOCALIZATION_ABORTED)
                    {
                        reloc_resp.status = "RelocalizationNone";
                    }
                    else
                        reloc_resp.status = "RelocalizationCanceled";
                    break;
                }
                default:
                    break;
                }
            }

            lastRelocalizationStatus_ = reloc_status.status;
            pubRelocalizaitonStatus_->publish(reloc_resp);
        }
    }

    //////////////////////////////////////////////////////////////////////////
    ServerStereoImageWorker::ServerStereoImageWorker(SlamwareRosSdkServer *pRosSdkServer,
                                                     const std::string &wkName,
                                                     const std::chrono::milliseconds &triggerInterval)
        : super_t(pRosSdkServer, wkName, triggerInterval),lastTimestamp_(0)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubLeftImage_ = nhRos->create_publisher<sensor_msgs::msg::Image>(srvParams.getParameter<std::string>("left_image_raw_topic_name"), 5);
        pubRightImage_ = nhRos->create_publisher<sensor_msgs::msg::Image>(srvParams.getParameter<std::string>("right_image_raw_topic_name"), 5);
        pubStereoKeyPoints_ = nhRos->create_publisher<sensor_msgs::msg::Image>(srvParams.getParameter<std::string>("stereo_keypoints_topic_name"), 5);
    }

    ServerStereoImageWorker::~ServerStereoImageWorker()
    {
    }

    bool ServerStereoImageWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }

    void ServerStereoImageWorker::doPerform()
    {
        // Fill leftImage and rightImage with actual data from Aurora SDK
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        RemoteTrackingFrameInfo trackingFrame;
        //filter outdate frame
        if (auroraSDK->dataProvider.peekTrackingData(trackingFrame))
        {
            if(trackingFrame.trackingInfo.timestamp_ns>lastTimestamp_)
            {
                lastTimestamp_ = trackingFrame.trackingInfo.timestamp_ns;
            }
            else
            {
                return;
            }
        }
       else
       {
            return;
       }

        cv::Mat left, right;
        trackingFrame.leftImage.toMat(left);
        trackingFrame.rightImage.toMat(right);

        // Check if images are empty before processing
        if (left.empty() || right.empty()) {
            return;
        }

        // Convert BGR or GRAY to RGB
        if(left.channels()==3)
            cv::cvtColor(left, left, cv::COLOR_BGR2RGB);
        else if(left.channels()==1)
            cv::cvtColor(left, left, cv::COLOR_GRAY2RGB);
        else
            return;
        const auto& srvParams = serverParams();
        rclcpp::Time pair_stamp = rosNodeHandle()->get_clock()->now();
        std_msgs::msg::Header header_pair;
        header_pair.stamp = pair_stamp;

        if(!srvParams.getParameter<bool>("raw_image_on"))
        {
            header_pair.frame_id = srvParams.getParameter<std::string>("camera_left");
            cv_bridge::CvImage img_bridge_left(header_pair, sensor_msgs::image_encodings::RGB8, left);
            pubLeftImage_->publish(*img_bridge_left.toImageMsg());
        }

        if(right.channels()==3)
            cv::cvtColor(right, right, cv::COLOR_BGR2RGB);
        else if(right.channels()==1)
            cv::cvtColor(right, right, cv::COLOR_GRAY2RGB);
        else
            return;

        if(!srvParams.getParameter<bool>("raw_image_on"))
        {
            header_pair.frame_id = srvParams.getParameter<std::string>("camera_right");
            cv_bridge::CvImage img_bridge_right(header_pair, sensor_msgs::image_encodings::RGB8, right);
            pubRightImage_->publish(*img_bridge_right.toImageMsg());
        }


        // Get left and right keypoints from the Aurora SDK
        for (size_t i = 0; i < trackingFrame.getKeypointsLeftCount(); i++)
        {
            auto &kp = trackingFrame.getKeypointsLeftBuffer()[i];
            if (kp.flags)
                cv::circle(left, cv::Point(kp.x, kp.y), 2, cv::Scalar(0, 255, 0), 2);
        }
        for (size_t i = 0; i < trackingFrame.getKeypointsRightCount(); i++)
        {
            auto &kp = trackingFrame.getKeypointsRightBuffer()[i];
            if (kp.flags)
                cv::circle(right, cv::Point(kp.x, kp.y), 2, cv::Scalar(0, 255, 0), 2);
        }

        cv::Mat merged;
        cv::hconcat(left, right, merged);
        std_msgs::msg::Header header_stereo_keypoints;
        cv_bridge::CvImage img_bridge_stereo;
        header_stereo_keypoints.frame_id = srvParams.getParameter<std::string>("camera_left");
        header_stereo_keypoints.stamp = rclcpp::Clock().now();
        img_bridge_stereo = cv_bridge::CvImage(header_stereo_keypoints, sensor_msgs::image_encodings::RGB8, merged);
        sensor_msgs::msg::Image::SharedPtr mergeImage = img_bridge_stereo.toImageMsg();
        pubStereoKeyPoints_->publish(*mergeImage);
    }

    //////////////////////////////////////////////////////////////////////////
    ServerStereoCameraInfoWorker::ServerStereoCameraInfoWorker(SlamwareRosSdkServer* pRosSdkServer,
                                                              const std::string& wkName,
                                                              const std::chrono::milliseconds& triggerInterval)
        : super_t(pRosSdkServer, wkName, triggerInterval)
        , calibrationReceived_(false)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubLeftCameraInfo_ = nhRos->create_publisher<sensor_msgs::msg::CameraInfo>(
            srvParams.getParameter<std::string>("camera_left_camera_info_topic"), 10);
        pubRightCameraInfo_ = nhRos->create_publisher<sensor_msgs::msg::CameraInfo>(
            srvParams.getParameter<std::string>("camera_right_camera_info_topic"), 10);
        memset(&lastCalib_, 0, sizeof(lastCalib_));
    }

    ServerStereoCameraInfoWorker::~ServerStereoCameraInfoWorker()
    {
    }

    bool ServerStereoCameraInfoWorker::reinitWorkLoop()
    {
        calibrationReceived_ = false;
        return true;
    }

    void ServerStereoCameraInfoWorker::fillCameraInfoFromCalibration(
        const slamtec_aurora_sdk_single_camera_calibration_t& calib,
        const slamtec_aurora_sdk_ext_camera_transform_t* extrinsics,
        int camIndex,
        sensor_msgs::msg::CameraInfo& cameraInfo,
        const std::string& frameId)
    {
        cameraInfo.header.stamp = rclcpp::Clock().now();
        cameraInfo.header.frame_id = frameId;
        cameraInfo.height = static_cast<uint32_t>(calib.height > 0 ? calib.height : 480);
        cameraInfo.width = static_cast<uint32_t>(calib.width > 0 ? calib.width : 640);

        // K: 3x3 intrinsics (row-major) [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        cameraInfo.k[0] = calib.intrinsics[0];  // fx
        cameraInfo.k[1] = 0.0;
        cameraInfo.k[2] = calib.intrinsics[2];  // cx
        cameraInfo.k[3] = 0.0;
        cameraInfo.k[4] = calib.intrinsics[1];  // fy
        cameraInfo.k[5] = calib.intrinsics[3];  // cy
        cameraInfo.k[6] = 0.0;
        cameraInfo.k[7] = 0.0;
        cameraInfo.k[8] = 1.0;

        // D: distortion. SDK has 5 coeffs; plumb_bob uses k1,k2,p1,p2,k3.
        cameraInfo.d.resize(5);
        cameraInfo.d[0] = calib.distortion[0];
        cameraInfo.d[1] = calib.distortion[1];
        cameraInfo.d[2] = (calib.distortion[2] != 0.0f || calib.distortion[3] != 0.0f) ? calib.distortion[2] : 0.0;
        cameraInfo.d[3] = (calib.distortion[2] != 0.0f || calib.distortion[3] != 0.0f) ? calib.distortion[3] : 0.0;
        cameraInfo.d[4] = calib.distortion[4];

        if (calib.len_type == SLAMTEC_AURORA_SDK_LEN_TYPE_KANNALABRANDT) {
            cameraInfo.distortion_model = "rational_polynomial";
        } else {
            cameraInfo.distortion_model = "plumb_bob";
        }

        // R: rectification matrix (identity for unrectified)
        cameraInfo.r[0] = 1.0; cameraInfo.r[1] = 0.0; cameraInfo.r[2] = 0.0;
        cameraInfo.r[3] = 0.0; cameraInfo.r[4] = 1.0; cameraInfo.r[5] = 0.0;
        cameraInfo.r[6] = 0.0; cameraInfo.r[7] = 0.0; cameraInfo.r[8] = 1.0;

        float fx = calib.intrinsics[0];
        float fy = calib.intrinsics[1];
        float cx = calib.intrinsics[2];
        float cy = calib.intrinsics[3];
        float Tx = 0.0f;

        if (extrinsics && camIndex == 1) {
            // Right camera: P(0,3) = -fx * baseline. 4x4 row-major: X translation at index 3
            Tx = extrinsics->t_c2_c1[3];
        }

        // P: 3x4 projection matrix (row-major)
        cameraInfo.p[0] = fx;  cameraInfo.p[1] = 0.0; cameraInfo.p[2] = cx;  cameraInfo.p[3] = -fx * Tx;
        cameraInfo.p[4] = 0.0; cameraInfo.p[5] = fy;  cameraInfo.p[6] = cy;  cameraInfo.p[7] = 0.0;
        cameraInfo.p[8] = 0.0; cameraInfo.p[9] = 0.0; cameraInfo.p[10] = 1.0; cameraInfo.p[11] = 0.0;
    }

    void ServerStereoCameraInfoWorker::doPerform()
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK) {
            return;
        }

        slamtec_aurora_sdk_camera_calibration_t calib;
        slamtec_aurora_sdk_errorcode_t err = SLAMTEC_AURORA_SDK_ERRORCODE_OK;
        if (!auroraSDK->dataProvider.getCameraCalibration(calib, &err)) {
            if (!calibrationReceived_) {
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ServerStereoCameraInfoWorker"),
                    *rosSdkServer()->get_clock(), 5000,
                    "Failed to get camera calibration: error %d", static_cast<int>(err));
            }
            return;
        }

        calibrationReceived_ = true;
        lastCalib_ = calib;

        const auto& srvParams = serverParams();
        std::string leftFrame = srvParams.getParameter<std::string>("camera_left");
        std::string rightFrame = srvParams.getParameter<std::string>("camera_right");

        sensor_msgs::msg::CameraInfo leftInfo, rightInfo;

        fillCameraInfoFromCalibration(
            calib.camera_calibration[0],
            (calib.camera_type == SLAMTEC_AURORA_SDK_CAMERA_TYPE_STEREO && calib.camera_calibration[1].width > 0)
                ? &calib.ext_camera_transform[1] : nullptr,
            0,
            leftInfo,
            leftFrame);

        if (calib.camera_type == SLAMTEC_AURORA_SDK_CAMERA_TYPE_STEREO && calib.camera_calibration[1].width > 0) {
            fillCameraInfoFromCalibration(
                calib.camera_calibration[1],
                &calib.ext_camera_transform[1],
                1,
                rightInfo,
                rightFrame);
            pubRightCameraInfo_->publish(rightInfo);
        }

        pubLeftCameraInfo_->publish(leftInfo);
    }
    
    ServerImuRawDataWorker::ServerImuRawDataWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
        , lastTimestamp_(0)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubImuRawData_ = nhRos->create_publisher<sensor_msgs::msg::Imu>(srvParams.getParameter<std::string>("imu_raw_data_topic"), 10);
        //convert g to m/s2
        acc_scale_ = 9.806649344;
        // convert dps to rad/s
        gyro_scale_ = M_PI/180;
    }

    ServerImuRawDataWorker::~ServerImuRawDataWorker()
    {
        //
    }

    bool ServerImuRawDataWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }

    void ServerImuRawDataWorker::doPerform()
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "Failed to get aurora sdk");
            return;
        }

        std::vector<slamtec_aurora_sdk_imu_data_t> imuData;
        const auto& srvParams = serverParams();
        if (auroraSDK->dataProvider.peekIMUData(imuData))
        {
            for (auto &imu : imuData)
            {
                if (imu.timestamp_ns <= lastTimestamp_)
                {
                    // ignore the old data that has been fetched before
                    continue;
                }
                lastTimestamp_ = imu.timestamp_ns;
                sensor_msgs::msg::Imu imuRawDataRos;
                imuRawDataRos.linear_acceleration.x = imu.acc[0]*acc_scale_;
                imuRawDataRos.linear_acceleration.y = imu.acc[1]*acc_scale_;
                imuRawDataRos.linear_acceleration.z = imu.acc[2]*acc_scale_;
                imuRawDataRos.angular_velocity.x = imu.gyro[0]*gyro_scale_;
                imuRawDataRos.angular_velocity.y = imu.gyro[1]*gyro_scale_;
                imuRawDataRos.angular_velocity.z = imu.gyro[2]*gyro_scale_;
                imuRawDataRos.header.stamp = rclcpp::Clock().now();
                imuRawDataRos.header.frame_id = srvParams.getParameter<std::string>("imu_frame");
                pubImuRawData_->publish(imuRawDataRos);
            }
        }
    }

    ServerPointCloudWorker::ServerPointCloudWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubPointCloud_ = nhRos->create_publisher<sensor_msgs::msg::PointCloud2>(srvParams.getParameter<std::string>("point_cloud_topic_name"), 5);
    }

    ServerPointCloudWorker::~ServerPointCloudWorker()
    {
        //
    }

    bool ServerPointCloudWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }

    void ServerPointCloudWorker::doPerform()
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "Failed to get aurora sdk");
            return;
        }

        int activeMapId = -1;
        slamtec_aurora_sdk_map_desc_t activeMapDesc;
        std::vector<slamtec_aurora_sdk_map_point_desc_t> mapPoints;
        slamtec_aurora_sdk_global_map_desc_t globalMapDesc;
        if(!auroraSDK->dataProvider.getGlobalMappingInfo(globalMapDesc))
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "Failed to get global mapping info");
            return;
        }

        activeMapId = globalMapDesc.activeMapID;
        // to access the map data, we need to create a map data visitor
        RemoteMapDataVisitor mapDataVisitor;
        mapDataVisitor.subscribeMapPointData([&](const slamtec_aurora_sdk_map_point_desc_t &mapPointDesc)
                                             {
                                            // handle the map point data
                                            mapPoints.push_back(mapPointDesc); 
                                            });

        // start the accessing session, this will block the background syncing thread during the accessing process.
        if (!auroraSDK->dataProvider.accessMapData(mapDataVisitor, {(uint32_t)activeMapId}))
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "Failed to start the accessing session");
            return;
        }

        // tf_vslam_to_ros.setValue(0, 1, 0,
        //                        -1, 0, 0,
        //                        0, 0, 1);
        const tf2::Matrix3x3 tf_vslam_to_ros(1, 0, 0,
                                             0, 1, 0,
                                             0, 0, 1);
        // publish map point to sensor_msgs::msg::PointCloud2
        const auto& srvParams = serverParams();
        const int num_channels = 3; // x y z
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = rclcpp::Clock().now();
        cloud.header.frame_id = srvParams.getParameter<std::string>("map_frame");
        cloud.height = 1;
        cloud.width = mapPoints.size();
        cloud.is_dense = true;
        cloud.is_bigendian = false;
        cloud.point_step = num_channels * sizeof(float);
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.fields.resize(num_channels);

        std::string channel_id[] = {"x", "y", "z"};
        for (int i = 0; i < num_channels; i++)
        {
            cloud.fields[i].name = channel_id[i];
            cloud.fields[i].offset = i * sizeof(float);
            cloud.fields[i].count = 1;
            cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        }

        cloud.data.resize(cloud.row_step * cloud.height);

        // mapPoints to cloud
        unsigned char *cloud_data_ptr = &(cloud.data[0]);

        float data_array[num_channels];
        for (unsigned int i = 0; i < cloud.width; i++)
        {
            //map points from orbslam2 aixs to ros axis
            tf2::Vector3 mapPoint(mapPoints[i].position.x, mapPoints[i].position.y, mapPoints[i].position.z);
            tf2::Vector3 mapPoint_ros = tf_vslam_to_ros * mapPoint;
            data_array[0] = mapPoint_ros.x();
            data_array[1] = mapPoint_ros.y();
            data_array[2] = mapPoint_ros.z();
            memcpy(cloud_data_ptr + (i * cloud.point_step),
                   data_array, num_channels * sizeof(float));
        }

        pubPointCloud_->publish(cloud);
    }

    //////////////////////////////////////////////////////////////////////////

    RosConnectWorker::RosConnectWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const std::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        auto nhRos = rosNodeHandle();
        pubRosConnect_ = nhRos->create_publisher<std_msgs::msg::String>("/slamware_ros_sdk_server_node/state", 1);
    }

    RosConnectWorker::~RosConnectWorker()
    {
        //
    }

    bool RosConnectWorker::reinitWorkLoop()
    {
        if (!this->super_t::reinitWorkLoop())
            return false;

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void RosConnectWorker::doPerform()
    {
        auto connectStatus = std_msgs::msg::String();
        connectStatus.data = "connected";
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            connectStatus.data = "disconnected";
        }
        pubRosConnect_->publish(connectStatus);
    }

    //////////////////////////////////////////////////////////////////////////
    
    ServerEnhancedImagingWorker::ServerEnhancedImagingWorker(SlamwareRosSdkServer *pRosSdkServer,
                                                             const std::string &wkName,
                                                             const std::chrono::milliseconds &triggerInterval)
        : super_t(pRosSdkServer, wkName, triggerInterval)
        , depthCameraSupported_(false)
        , semanticSegmentationSupported_(false)
        , isInitialized_(false)
        , depth_lastTimestamp_(0),segmentation_lastTimestamp_(0)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        
        // Initialize depth camera publishers
        pubDepthImage_ = nhRos->create_publisher<sensor_msgs::msg::Image>(srvParams.getParameter<std::string>("depth_image_raw_topic_name"), 5);
        pubDepthColorized_ = nhRos->create_publisher<sensor_msgs::msg::Image>(srvParams.getParameter<std::string>("depth_image_colorized_topic_name"), 5);
                
        // Initialize semantic segmentation publishers
        pubSemanticSegmentation_ = nhRos->create_publisher<sensor_msgs::msg::Image>(srvParams.getParameter<std::string>("semantic_segmentation_topic_name"), 5);
        pubSemanticLabels_ = nhRos->create_publisher<sensor_msgs::msg::Image>(srvParams.getParameter<std::string>("semantic_labels_topic_name"), 5);
    }

    ServerEnhancedImagingWorker::~ServerEnhancedImagingWorker()
    {
    }

    bool ServerEnhancedImagingWorker::reinitWorkLoop()
    {
        if (!this->super_t::reinitWorkLoop())
            return false;

        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK) {
            isWorkLoopInitOk_ = false;
            return false;
        }

        // Check depth camera support
        depthCameraSupported_ = auroraSDK->enhancedImaging.isDepthCameraSupported();
        if (depthCameraSupported_) {
            auroraSDK->controller.setEnhancedImagingSubscription(SLAMTEC_AURORA_SDK_ENHANCED_IMAGE_TYPE_DEPTH, true);
            // Enable depth camera post-filtering (refines depth data; flags currently ignored by SDK)
            auroraSDK->enhancedImaging.setDepthCameraPostFiltering(true, 0);
            RCLCPP_INFO(rosNodeHandle()->get_logger(), "Depth camera supported, subscription enabled, post-filtering on");
        } else {
            RCLCPP_WARN(rosNodeHandle()->get_logger(), "Depth camera not supported");
        }

        // Check semantic segmentation support
        semanticSegmentationSupported_ = auroraSDK->enhancedImaging.isSemanticSegmentationReady();
        if (semanticSegmentationSupported_) {
            auroraSDK->controller.setEnhancedImagingSubscription(SLAMTEC_AURORA_SDK_ENHANCED_IMAGE_TYPE_SEMANTIC, true);
            RCLCPP_INFO(rosNodeHandle()->get_logger(), "Semantic segmentation supported and subscription enabled");
            
            // Get semantic segmentation label information
            std::string labelSetName;
            if (auroraSDK->enhancedImaging.getSemanticSegmentationLabelSetName(labelSetName)) {
                RCLCPP_INFO(rosNodeHandle()->get_logger(), "Semantic segmentation label set: %s", labelSetName.c_str());
            }
            
            slamtec_aurora_sdk_semantic_segmentation_label_info_t labelInfo;
            if (auroraSDK->enhancedImaging.getSemanticSegmentationLabels(labelInfo)) {
                for (int i = 0; i < labelInfo.label_count; ++i) {
                    RCLCPP_INFO(rosNodeHandle()->get_logger(), "Label ID: %d, Name: %s", i, labelInfo.label_names[i].name);
                }
                // Generate class colors
                generateClassColors(labelInfo.label_count);
                RCLCPP_INFO(rosNodeHandle()->get_logger(), "Class colors generated");
            }
        } else {
            RCLCPP_WARN(rosNodeHandle()->get_logger(), "Semantic segmentation not supported");
        }

        isInitialized_ = true;
        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerEnhancedImagingWorker::doPerform()
    {
        if (!isInitialized_) {
            return;
        }

        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK) {
            return;
        }

        auto currentTime = rclcpp::Clock().now();
        std_msgs::msg::Header header;
        header.stamp = currentTime;
        header.frame_id = "camera_depth_optical_frame";

        // Process depth camera data
        if (depthCameraSupported_) {
            processDepthCamera(header);
        }

        // Process semantic segmentation data
        if (semanticSegmentationSupported_) {
            processSemanticSegmentation(header);
        }
    }

    void ServerEnhancedImagingWorker::processDepthCamera(const std_msgs::msg::Header& header)
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK) {
            return;
        }

        RemoteEnhancedImagingFrame depthFrame;
             
        // Early return if cannot peek frame
        if (auroraSDK->enhancedImaging.peekDepthCameraFrame(depthFrame, SLAMTEC_AURORA_SDK_DEPTHCAM_FRAME_TYPE_DEPTH_MAP)) 
        {
            if(depthFrame.desc.timestamp_ns>depth_lastTimestamp_)
            {
                depth_lastTimestamp_ = depthFrame.desc.timestamp_ns;
            }
            else
            {
                return;
            }
        }
        else
        {
            return;
        }
        
        cv::Mat depthImg;
        depthFrame.image.toMat(depthImg);
        
        // Publish raw depth image
        cv_bridge::CvImage depthBridge(header, sensor_msgs::image_encodings::TYPE_32FC1, depthImg);
        pubDepthImage_->publish(*depthBridge.toImageMsg());
        
        // Create and publish colorized depth image
        cv::Mat heatmap;
        depthImg.convertTo(heatmap, CV_8UC1, -255.0 / 10.0f, 255);
        cv::Mat depthColorized;
        cv::applyColorMap(heatmap, depthColorized, cv::COLORMAP_JET);
        
        cv_bridge::CvImage depthColorizedBridge(header, sensor_msgs::image_encodings::BGR8, depthColorized);
        pubDepthColorized_->publish(*depthColorizedBridge.toImageMsg());
    }

    void ServerEnhancedImagingWorker::processSemanticSegmentation(const std_msgs::msg::Header& header)
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK) {
            return;
        }

        RemoteEnhancedImagingFrame segFrame;
        
        //filter outdate frame
        if (auroraSDK->enhancedImaging.peekSemanticSegmentationFrame(segFrame))
        {
            if(segFrame.desc.timestamp_ns>segmentation_lastTimestamp_)
            {
                segmentation_lastTimestamp_ = segFrame.desc.timestamp_ns;
            }
            else
            {
                return;
            }
        }
        else
        {
            return;
        }

        cv::Mat localSegMap;
        segFrame.image.toMat(localSegMap);
        auto currentSegMap = localSegMap.clone();

        if (currentSegMap.empty()) {
           return; 
        }

        // Publish raw label map (mono8, pixel value = COCO80 class ID) for semantic fusion
        cv_bridge::CvImage labelsBridge(header, sensor_msgs::image_encodings::MONO8, currentSegMap);
        pubSemanticLabels_->publish(*labelsBridge.toImageMsg());
        
        // Peek camera preview image for overlay (if needed)
        RemoteStereoImagePair cameraImagePair;
        cv::Mat currentCameraImage;
        
        if (auroraSDK->dataProvider.peekCameraPreviewImage(cameraImagePair, segFrame.desc.timestamp_ns, true)) {
            cameraImagePair.leftImage.toMat(currentCameraImage);
            if (currentCameraImage.channels() == 1) {
                cv::cvtColor(currentCameraImage, currentCameraImage, cv::COLOR_GRAY2BGR);
            }
        }

        auto currentColorizedSegMap = colorizeSegmentationMap(currentSegMap);
        auto currentOverlayBase = createCameraOverlay(currentCameraImage, currentColorizedSegMap);

        // Publish colorized segmentation
        cv_bridge::CvImage segBridge(header, sensor_msgs::image_encodings::BGR8, currentOverlayBase);
        pubSemanticSegmentation_->publish(*segBridge.toImageMsg());
    }

    cv::Mat ServerEnhancedImagingWorker::createCameraOverlay(const cv::Mat& cameraImage, const cv::Mat& colorizedSegMap)
    {
        if (cameraImage.empty() || colorizedSegMap.empty()) {
            return colorizedSegMap.clone();
        }
        
        cv::Mat overlay = cameraImage.clone();
        
        // Optimized blending using OpenCV vectorized operations
        cv::Mat mask;
        cv::inRange(colorizedSegMap, cv::Scalar(1, 1, 1), cv::Scalar(255, 255, 255), mask);
        
        // Blend only non-black pixels
        cv::Mat blendedSeg;
        cv::addWeighted(cameraImage, 0.6, colorizedSegMap, 0.4, 0, blendedSeg);
        blendedSeg.copyTo(overlay, mask);
        
        return overlay;
    }

    cv::Mat ServerEnhancedImagingWorker::colorizeSegmentationMap(const cv::Mat& segMap)
    {
        cv::Mat colorized(segMap.size(), CV_8UC3);
        for (int y = 0; y < segMap.rows; y++) {
            for (int x = 0; x < segMap.cols; x++) {
                uint8_t classId = segMap.at<uint8_t>(y, x);
                if (classId < classColors_.size()) {
                    colorized.at<cv::Vec3b>(y, x) = classColors_[classId];
                } else {
                    colorized.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128); // Gray for unknown classes
                }
            }
        }
        
        return colorized;
    }

    void ServerEnhancedImagingWorker::generateClassColors(int numClasses) {
        classColors_.clear();
        classColors_.resize(numClasses);
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(50, 255);
        
        // Set background (index 0) to black for transparency effect
        classColors_[0] = cv::Vec3b(0, 0, 0);
        
        // Generate random colors for other classes
        for (int i = 1; i < numClasses; i++) {
            classColors_[i] = cv::Vec3b(dis(gen), dis(gen), dis(gen));
        }
    }

}
