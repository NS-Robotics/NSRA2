#ifndef NSSC_TRIANGULATION_
#define NSSC_TRIANGULATION_

#include "node.h"
#include "nssc_errors.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>

class Triangulation
{
public:
    Triangulation(std::shared_ptr<NSSC> &node, char *setName);
    NSSC_STATUS findOrigin();

private:
    std::shared_ptr<NSSC> node;
    std::string msgCaller = "Triangulation";

    std::string setPath;

    cv::Mat left_K, right_K, left_D, right_D;
    cv::Mat PR, PL;
};

#endif //NSSC_TRIANGULATION_