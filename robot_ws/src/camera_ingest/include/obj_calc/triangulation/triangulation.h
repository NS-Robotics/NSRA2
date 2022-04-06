#ifndef NSSC_TRIANGULATION_
#define NSSC_TRIANGULATION_

#include "node.h"
#include "nssc_errors.h"
#include "frame_manager.h"
#include "stereo_frame.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>

#include <fstream>
#include <Eigen/Dense>

namespace nssc
{
    namespace process
    {
        class Triangulation
        {
        public:
            Triangulation(std::shared_ptr<ros::NSSC> &node, std::unique_ptr<send::FrameManager>* frameManager, const char *setName);
            NSSC_STATUS init();
            NSSC_STATUS findOrigin();

        protected:
            std::vector<Eigen::Vector3d> _transform_coordinates(const std::vector<Eigen::Vector3d>& inp);

            std::shared_ptr<ros::NSSC> node;
            std::unique_ptr<send::FrameManager>* frameManager;

            cv::Mat left_map1, left_map2, right_map1, right_map2;
            cv::Mat RR, RL, PR, PL;
            cv::Mat left_K, right_K, left_D, right_D;
            cv::Size mono_size;

            Eigen::Matrix3d rotation_mtrx;
            Eigen::Vector3d origin;
            std::vector<Eigen::Vector3d> origin_vec;
            std::vector<cv::Point2f> left_origin_pts;
            std::vector<cv::Point2f> right_origin_pts;

        private:
            std::string msgCaller = "Triangulation";

            bool __originMarkersExist(std::vector<int> ids);

            std::string setPath;

            cv::Ptr<cv::aruco::Dictionary> dictionary;
            cv::Ptr<cv::aruco::DetectorParameters> parameters;
        };
    }
}

#endif //NSSC_TRIANGULATION_