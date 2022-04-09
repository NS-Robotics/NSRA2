#include "stereo_frame.h"
#include "node.h"
#include "mono_frame.h"

class RGBAStereoFrame : public nssc::framestruct::StereoFrame
{
public:
    void convert(nssc::framestruct::MonoFrame *leftCamera, nssc::framestruct::MonoFrame *rightCamera, bool resize) override
    {
        this->left_camera = leftCamera;
        this->right_camera = rightCamera;

        process(resize);

        auto c_timedif = std::chrono::duration_cast<std::chrono::microseconds>(
                leftCamera->rgba_buf.timestamp - rightCamera->rgba_buf.timestamp);
        this->timedif = std::abs(c_timedif.count());

        if (this->timedif > this->node->g_config.frame_config.max_frame_time_diff)
        {
            this->node->printWarning(this->msg_caller,
                                     "Frame timestamp difference out of bounds: " + std::to_string(this->timedif) +
                                     " us");
        }
    }

    void process(bool resize) override
    {
        cv::cuda::GpuMat dMergedFrame(cv::Size(this->node->g_config.frame_config.stereo_x_res,
                                               this->node->g_config.frame_config.stereo_y_res), CV_8UC4,
                                      this->concatenate_buf.dImageBuf);

        cv::cuda::GpuMat dRGBAImageBufRight(cv::Size(this->node->g_config.frame_config.mono_x_res,
                                                     this->node->g_config.frame_config.mono_y_res), CV_8UC4,
                                            right_camera->rgba_buf.dImageBuf);
        cv::cuda::GpuMat dRGBAImageBufLeft(cv::Size(this->node->g_config.frame_config.mono_x_res,
                                                    this->node->g_config.frame_config.mono_y_res), CV_8UC4,
                                           left_camera->rgba_buf.dImageBuf);

        dRGBAImageBufRight.copyTo(dMergedFrame.operator()(cv::Rect(0, 0, dRGBAImageBufRight.cols,
                                                                   dRGBAImageBufRight.rows)));
        dRGBAImageBufLeft.copyTo(dMergedFrame.operator()(cv::Rect(dRGBAImageBufRight.cols, 0,
                                                                  dRGBAImageBufLeft.cols, dRGBAImageBufLeft.rows)));

        this->stereo_buf = &this->concatenate_buf;

        if (resize)
        {
            cv::cuda::GpuMat inputFrame(cv::Size(this->node->g_config.frame_config.stereo_x_res,
                                                 this->node->g_config.frame_config.stereo_y_res), CV_8UC4,
                                        this->concatenate_buf.dImageBuf);
            cv::cuda::GpuMat resizedFrame(cv::Size(this->node->g_config.frame_config.resize_x_res,
                                                   this->node->g_config.frame_config.resize_y_res), CV_8UC4,
                                          this->resize_buf.dImageBuf);

            cv::cuda::resize(inputFrame, resizedFrame, cv::Size(this->node->g_config.frame_config.resize_x_res,
                                                                this->node->g_config.frame_config.resize_y_res));

            this->stereo_buf = &this->resize_buf;
        }
    }

    void alloc(std::shared_ptr<nssc::ros::NSSC> &node) override
    {
        this->node = node;

        cudaSetDeviceFlags(cudaDeviceMapHost);
        cudaHostAlloc((void **) &this->concatenate_buf.hImageBuf, this->node->g_config.frame_config.stereo_buf_size,
                      cudaHostAllocMapped);
        cudaHostGetDevicePointer((void **) &this->concatenate_buf.dImageBuf, (void *) this->concatenate_buf.hImageBuf, 0);

        cudaSetDeviceFlags(cudaDeviceMapHost);
        cudaHostAlloc((void **) &this->resize_buf.hImageBuf, this->node->g_config.frame_config.resize_buf_size,
                      cudaHostAllocMapped);
        cudaHostGetDevicePointer((void **) &this->resize_buf.dImageBuf, (void *) this->resize_buf.hImageBuf, 0);
    }

    ~RGBAStereoFrame()
    {
        cudaFreeHost(this->concatenate_buf.hImageBuf);
        cudaFreeHost(this->resize_buf.hImageBuf);
    }
};

class I420StereoFrame : public nssc::framestruct::StereoFrame
{
public:
    void convert(nssc::framestruct::MonoFrame *leftCamera, nssc::framestruct::MonoFrame *rightCamera, bool resize) override
    {
        this->left_camera = leftCamera;
        this->right_camera = rightCamera;

        cv::cuda::GpuMat dMergedFrame(cv::Size(this->node->g_config.frame_config.stereo_x_res,
                                               this->node->g_config.frame_config.stereo_y_res), CV_8UC1,
                                      this->concatenate_buf.dImageBuf);

        cv::cuda::GpuMat dRGBAImageBufRight(cv::Size(this->node->g_config.frame_config.mono_x_res,
                                                     this->node->g_config.frame_config.mono_y_res), CV_8UC1,
                                            rightCamera->rgba_buf.dImageBuf);
        cv::cuda::GpuMat dRGBAImageBufLeft(cv::Size(this->node->g_config.frame_config.mono_x_res,
                                                    this->node->g_config.frame_config.mono_y_res), CV_8UC1,
                                           leftCamera->rgba_buf.dImageBuf);

        dRGBAImageBufRight.copyTo(
                dMergedFrame.operator()(cv::Rect(0, 0, dRGBAImageBufRight.cols, dRGBAImageBufRight.rows)));
        dRGBAImageBufLeft.copyTo(dMergedFrame.operator()(
                cv::Rect(dRGBAImageBufRight.cols, 0, dRGBAImageBufLeft.cols, dRGBAImageBufLeft.rows)));

        this->stereo_buf = &this->concatenate_buf;

        auto c_timedif = std::chrono::duration_cast<std::chrono::microseconds>(
                leftCamera->rgba_buf.timestamp - rightCamera->rgba_buf.timestamp);
        this->timedif = std::abs(c_timedif.count());
    }

    void process(bool resize) override
    {

    }

    void alloc(std::shared_ptr<nssc::ros::NSSC> &node) override
    {
        this->node = node;

        cudaSetDeviceFlags(cudaDeviceMapHost);
        cudaHostAlloc((void **) &this->concatenate_buf.hImageBuf, this->node->g_config.frame_config.stereo_buf_size,
                      cudaHostAllocMapped);
        cudaHostGetDevicePointer((void **) &this->concatenate_buf.dImageBuf, (void *) this->concatenate_buf.hImageBuf, 0);
    }

    ~I420StereoFrame()
    {
        cudaFreeHost(this->concatenate_buf.hImageBuf);
    }
};

class UYVYStereoFrame : public nssc::framestruct::StereoFrame
{
public:
    void convert(nssc::framestruct::MonoFrame *leftCamera, nssc::framestruct::MonoFrame *rightCamera, bool resize) override
    {
        this->left_camera = leftCamera;
        this->right_camera = rightCamera;

        cv::cuda::GpuMat dMergedFrame(cv::Size(this->node->g_config.frame_config.stereo_x_res,
                                               this->node->g_config.frame_config.stereo_y_res), CV_8UC1,
                                      this->concatenate_buf.dImageBuf);

        cv::cuda::GpuMat dRGBAImageBufRight(cv::Size(this->node->g_config.frame_config.mono_x_res,
                                                     this->node->g_config.frame_config.mono_y_res), CV_8UC1,
                                            rightCamera->rgba_buf.dImageBuf);
        cv::cuda::GpuMat dRGBAImageBufLeft(cv::Size(this->node->g_config.frame_config.mono_x_res,
                                                    this->node->g_config.frame_config.mono_y_res), CV_8UC1,
                                           leftCamera->rgba_buf.dImageBuf);

        dRGBAImageBufRight.copyTo(
                dMergedFrame.operator()(cv::Rect(0, 0, dRGBAImageBufRight.cols, dRGBAImageBufRight.rows)));
        dRGBAImageBufLeft.copyTo(dMergedFrame.operator()(
                cv::Rect(dRGBAImageBufRight.cols, 0, dRGBAImageBufLeft.cols, dRGBAImageBufLeft.rows)));

        this->stereo_buf = &this->concatenate_buf;

        auto c_timedif = std::chrono::duration_cast<std::chrono::microseconds>(
                leftCamera->rgba_buf.timestamp - rightCamera->rgba_buf.timestamp);
        this->timedif = std::abs(c_timedif.count());
    }

    void process(bool resize) override
    {

    }

    void alloc(std::shared_ptr<nssc::ros::NSSC> &node) override
    {
        this->node = node;

        cudaSetDeviceFlags(cudaDeviceMapHost);
        cudaHostAlloc((void **) &this->concatenate_buf.hImageBuf, this->node->g_config.frame_config.stereo_buf_size,
                      cudaHostAllocMapped);
        cudaHostGetDevicePointer((void **) &this->concatenate_buf.dImageBuf, (void *) this->concatenate_buf.hImageBuf, 0);
    }

    ~UYVYStereoFrame()
    {
        cudaFreeHost(this->concatenate_buf.hImageBuf);
    }
};

nssc::framestruct::StereoFrame *nssc::framestruct::StereoFrame::makeFrame(NSSC_FRAME_TYPE type)
{
    switch (type)
    {
        case NSSC_FRAME_RGBA:
            return new RGBAStereoFrame;
        case NSSC_FRAME_I420:
            return new I420StereoFrame;
        case NSSC_FRAME_UYVY:
            return new UYVYStereoFrame;
    }
}