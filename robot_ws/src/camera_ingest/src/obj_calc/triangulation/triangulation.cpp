#include <iostream>
#include "triangulation.h"

Triangulation::Triangulation(std::shared_ptr<NSSC> &node, std::unique_ptr<NDIframeManager>* frameManager,
                             const char *setName)
{
    this->node = node;
    this->frameManager = frameManager;
    this->setPath = this->node->g_config.share_dir + "/" + setName + "/";
}

NSSC_STATUS Triangulation::init()
{
    std::ifstream xmlFile(this->setPath + "config.xml");
    if (xmlFile.fail())
    {
        this->node->printWarning(this->msgCaller, "This configuration doesn't exist!");
        return NSSC_TRIANGULATION_FILE_NOT_FOUND;
    }
    else
    {
        cv::FileStorage config_file(this->setPath + "config.xml", cv::FileStorage::READ);

        config_file["left_K"] >> this->left_K;
        config_file["left_D"] >> this->left_D;
        config_file["right_K"] >> this->right_K;
        config_file["right_D"] >> this->right_D;

        config_file["RR"] >> this->RR;
        config_file["RL"] >> this->RL;
        config_file["PR"] >> this->PR;
        config_file["PL"] >> this->PL;

        this->dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        this->parameters = cv::aruco::DetectorParameters::create();

        this->mono_size = cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res);

        cv::initUndistortRectifyMap(this->left_K, this->left_D, this->RL, this->PL,
                                    mono_size, CV_32FC1, this->left_map1, this->left_map2);
        cv::initUndistortRectifyMap(this->right_K, this->right_D, this->RR, this->PR,
                                    mono_size, CV_32FC1, this->right_map1, this->right_map2);

        NSSC_STATUS status;
        this->node->printInfo(this->msgCaller, "Calculate origin");

        status = findOrigin();

        if (status == NSSC_STATUS_SUCCESS);
            this->node->printInfo(this->msgCaller, "Initialized!");

        return status;
    }
}

std::string vector_content(std::vector<int> v){
    std::string s;
    s += '[';
    for (int i = 0; i < v.size(); i++){
        s += "\"";
        s += std::to_string(v[i]) ;
        s += "\"";
        if (v[i] != v.back())
            s += " ,";
    }
    s += ']';
    return s;
}

NSSC_STATUS Triangulation::findOrigin()
{
    stereoFrame* stereoFrame;

    while (this->node->g_config.frameConfig.stream_on)
    {
        stereoFrame = (*this->frameManager)->getCameraFrame();
        if (stereoFrame->timedif < this->node->g_config.triangulationConfig.max_origin_frame_time_diff)
            break;
        else
            (*this->frameManager)->returnBuf(stereoFrame);
    }

    cv::Mat left_inp(mono_size,CV_8UC4, stereoFrame->leftCamera->frameBuf.hImageBuf);
    cv::Mat right_inp(mono_size, CV_8UC4, stereoFrame->rightCamera->frameBuf.hImageBuf);

    cv::Mat left_conv;
    cv::Mat right_conv;

    cv::cvtColor(left_inp, left_conv, cv::COLOR_RGBA2GRAY);
    cv::cvtColor(right_inp, right_conv, cv::COLOR_RGBA2GRAY);

    std::vector<int> left_markerIds, right_markerIds;
    std::vector<std::vector<cv::Point2f>> left_markerCorners, right_markerCorners, left_rejectedCandidates, right_rejectedCandidates;

    cv::aruco::detectMarkers(left_conv, this->dictionary, left_markerCorners,
                             left_markerIds,this->parameters, left_rejectedCandidates,
                             this->left_K, this->left_D);
    cv::aruco::detectMarkers(right_conv, this->dictionary, right_markerCorners,
                             right_markerIds,this->parameters, right_rejectedCandidates,
                             this->left_K, this->left_D);

    if (!__originMarkersExist(left_markerIds) || !__originMarkersExist(right_markerIds))
    {
        this->node->printError(this->msgCaller, "Origin markers not found!");
        return NSSC_TRIANGULATION_MARKERS_NOT_FOUND;
    }

    std::vector<cv::Point2f> left_originMarkers, right_originMarkers;
    for(int i = 0; i < 3; i++)
    {
        int left_idx, right_idx;
        auto left_element = std::find(left_markerIds.begin(), left_markerIds.end(), this->node->g_config.triangulationConfig.origin_ids[i]);
        left_idx = left_element - left_markerIds.begin();
        auto right_element = std::find(right_markerIds.begin(), right_markerIds.end(), this->node->g_config.triangulationConfig.origin_ids[i]);
        right_idx = right_element - right_markerIds.begin();
        left_originMarkers.push_back(left_markerCorners[left_idx][0]);
        right_originMarkers.push_back(right_markerCorners[right_idx][0]);
    }

    this->left_origin_pts = left_originMarkers;
    this->right_origin_pts = right_originMarkers;

    std::vector<cv::Point2f> left_2D_undistort, right_2D_undistort;
    cv::undistortPoints(left_originMarkers, left_2D_undistort, this->left_K, this->left_D, this->RL, this->PL);
    cv::undistortPoints(right_originMarkers, right_2D_undistort, this->right_K, this->right_D, this->RR, this->PR);

    cv::Mat p_4d_origin(4,3,CV_64F);
    cv::triangulatePoints(this->PL, this->PR, left_2D_undistort, right_2D_undistort, p_4d_origin);

    std::vector<Eigen::Vector3d> p_3d_origin;
    for (int i = 0; i < 3; i++)
    {
        p_3d_origin.emplace_back( p_4d_origin.at<float>(0,i) / p_4d_origin.at<float>(3,i),
                                  p_4d_origin.at<float>(1,i) / p_4d_origin.at<float>(3,i),
                                  p_4d_origin.at<float>(2,i) / p_4d_origin.at<float>(3,i) );
    }

    this->origin_vec.emplace_back(p_3d_origin[1] - p_3d_origin[0]);
    this->origin_vec.emplace_back(p_3d_origin[2] - p_3d_origin[0]);
    this->origin_vec.emplace_back(this->origin_vec[0].cross(this->origin_vec[1]));

    /*
    Eigen::Matrix4d transformation;
    transformation <<   1.0,                                 v_3d_origin[1][0]/v_3d_origin[1][1], v_3d_origin[2][0]/v_3d_origin[2][2], - p_3d_origin[0][0],
                        v_3d_origin[0][1]/v_3d_origin[0][0], 1.0,                                 v_3d_origin[2][1]/v_3d_origin[2][2], - p_3d_origin[0][1],
                        v_3d_origin[0][2]/v_3d_origin[0][0], v_3d_origin[1][2]/v_3d_origin[1][1], 1.0,                                 - p_3d_origin[0][2],
                                      0.0,               0.0,               0.0,               1.0;
    Eigen::Vector4d obj;
    obj << p_3d_origin[1][0], p_3d_origin[1][1], p_3d_origin[1][2], 1.0;
     */
    /*
    Eigen::Vector3d coords_new, unit_x_new, unit_y_new, unit_z_new;

    unit_x_new << v_3d_origin[0][0]/v_3d_origin[0][0], v_3d_origin[0][1]/v_3d_origin[0][0], v_3d_origin[0][2]/v_3d_origin[0][0];
    unit_y_new << v_3d_origin[1][0]/v_3d_origin[1][1], v_3d_origin[1][1]/v_3d_origin[1][1], v_3d_origin[1][2]/v_3d_origin[1][1];
    unit_z_new << v_3d_origin[2][0]/v_3d_origin[2][2], v_3d_origin[2][1]/v_3d_origin[2][2], v_3d_origin[2][2]/v_3d_origin[2][2];

    coords_new[0] = (p_3d_origin[1] - p_3d_origin[0]).dot(unit_x_new);
    coords_new[1] = (p_3d_origin[1] - p_3d_origin[0]).dot(unit_y_new);
    coords_new[2] = (p_3d_origin[1] - p_3d_origin[0]).dot(unit_z_new);

    std::cout << "new transformation" << std::endl << coords_new << std::endl;
    */
    this->rotation_mtrx <<  this->origin_vec[0][0], this->origin_vec[1][0], this->origin_vec[2][0],
                            this->origin_vec[0][1], this->origin_vec[1][1], this->origin_vec[2][1],
                            this->origin_vec[0][2], this->origin_vec[1][2], this->origin_vec[2][2];

    this->origin = p_3d_origin[0];

    cv::Mat left_out;
    cv::Mat right_out;
    cv::cvtColor(left_inp, left_out, cv::COLOR_RGBA2BGR);
    cv::cvtColor(left_inp, right_out, cv::COLOR_RGBA2BGR);

    cv::aruco::drawDetectedMarkers(left_out, left_markerCorners, left_markerIds);
    cv::aruco::drawDetectedMarkers(right_out, right_markerCorners, right_markerIds);

    cv::cvtColor(left_out, left_inp, cv::COLOR_BGR2RGBA);
    cv::cvtColor(right_out, right_inp, cv::COLOR_BGR2RGBA);

    (*this->frameManager)->sendFrame(stereoFrame);

    return NSSC_STATUS_SUCCESS;
}

std::vector<Eigen::Vector3d> Triangulation::_transform_coordinates(const std::vector<Eigen::Vector3d>& inp)
{
    Eigen::Vector3d obj_vec, mtrx_ret, obj;
    std::vector<Eigen::Vector3d> objects;

    for (auto & i : inp)
    {
        obj_vec = i - this->origin;
        mtrx_ret = this->rotation_mtrx.colPivHouseholderQr().solve(obj_vec);
        obj <<  (this->origin_vec[0] * mtrx_ret[0]).norm(),
                (this->origin_vec[1] * mtrx_ret[1]).norm(),
                (this->origin_vec[2] * mtrx_ret[2]).norm();
        objects.emplace_back(obj);
    }

    return objects;
}

bool Triangulation::__originMarkersExist(std::vector<int> ids)
{
    std::vector<int>& req_ids = this->node->g_config.triangulationConfig.origin_ids;

    return std::all_of(req_ids.begin(), req_ids.end(), [&ids](int a_elt) {
        return std::find(ids.begin(), ids.end(), a_elt) != ids.end();
    });
}
