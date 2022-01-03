#include "triangulation.h"

Triangulation::Triangulation(std::shared_ptr<NSSC> &node, char *setName)
{
    this->node = node;
    this->setPath = this->node->g_config.share_dir + "/" + setName + "/";

    cv::FileStorage config_file((this->setPath + "config.xml").c_str(), cv::FileStorage::READ);

    config_file["left_K"] >> this->left_K;
    config_file["left_D"] >> this->left_D;
    config_file["right_K"] >> this->right_K;
    config_file["right_D"] >> this->right_D;

    config_file["PR"] >> this->PR;
    config_file["PL"] >> this->PL;
}

NSSC_STATUS Triangulation::findOrigin()
{
    
}