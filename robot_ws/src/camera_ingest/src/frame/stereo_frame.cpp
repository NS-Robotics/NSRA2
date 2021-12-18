#include "stereo_frame.h"

class RGBAStereoFrame: public stereoFrame
{
    public:
        void convert(monoFrame* leftCamera, monoFrame* rightCamera)
        {
            this->leftCamera = leftCamera;
            this->rightCamera = rightCamera;

            cv::cuda::GpuMat dMergedFrame(cv::Size(this->node->g_config.stream_x_res, this->node->g_config.stream_y_res), CV_8UC4, stereoBuf.dImageBuf);

            cv::cuda::GpuMat dRGBAImageBufRight(cv::Size(this->node->g_config.mono_x_res, this->node->g_config.mono_y_res), CV_8UC4, leftCamera->frameBuf.dImageBuf);
            cv::cuda::GpuMat dRGBAImageBufLeft(cv::Size(this->node->g_config.mono_x_res, this->node->g_config.mono_y_res), CV_8UC4, rightCamera->frameBuf.dImageBuf);

            dRGBAImageBufRight.copyTo(dMergedFrame.operator()(cv::Rect(0, 0, dRGBAImageBufRight.cols, dRGBAImageBufRight.rows)));
            dRGBAImageBufLeft.copyTo(dMergedFrame.operator()(cv::Rect(dRGBAImageBufRight.cols, 0, dRGBAImageBufLeft.cols, dRGBAImageBufLeft.rows)));
            
            auto c_timedif = std::chrono::duration_cast<std::chrono::microseconds>(leftCamera->frameBuf.timestamp - rightCamera->frameBuf.timestamp);
            this->timedif = c_timedif.count();
        }

        void alloc(std::shared_ptr<NSSC>& node)
        {
            this->node = node;
            
            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->stereoBuf.hImageBuf, this->node->g_config.stereo_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->stereoBuf.dImageBuf, (void *) this->stereoBuf.hImageBuf , 0);
        }

        ~RGBAStereoFrame()
        {
            cudaFreeHost(this->stereoBuf.hImageBuf);
        }
};

class I420StereoFrame: public stereoFrame
{
    public:
        void convert(monoFrame* leftCamera, monoFrame* rightCamera)
        {
            this->leftCamera = leftCamera;
            this->rightCamera = rightCamera;

            cv::cuda::GpuMat dMergedFrame(cv::Size(this->node->g_config.stream_x_res, std::round(this->node->g_config.stream_y_res * 1.5)), CV_8UC1, stereoBuf.dImageBuf);

            cv::cuda::GpuMat dRGBAImageBufRight(cv::Size(this->node->g_config.mono_x_res, std::round(this->node->g_config.mono_y_res * 1.5)), CV_8UC1, rightCamera->frameBuf.dImageBuf);
            cv::cuda::GpuMat dRGBAImageBufLeft(cv::Size(this->node->g_config.mono_x_res, std::round(this->node->g_config.mono_y_res * 1.5)), CV_8UC1, leftCamera->frameBuf.dImageBuf);

            dRGBAImageBufRight.copyTo(dMergedFrame.operator()(cv::Rect(0, 0, dRGBAImageBufRight.cols, dRGBAImageBufRight.rows)));
            dRGBAImageBufLeft.copyTo(dMergedFrame.operator()(cv::Rect(dRGBAImageBufRight.cols, 0, dRGBAImageBufLeft.cols, dRGBAImageBufLeft.rows)));
            
            auto c_timedif = std::chrono::duration_cast<std::chrono::microseconds>(leftCamera->frameBuf.timestamp - rightCamera->frameBuf.timestamp);
            this->timedif = c_timedif.count();
        }

        void alloc(std::shared_ptr<NSSC>& node)
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->stereoBuf.hImageBuf, this->node->g_config.stereo_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->stereoBuf.dImageBuf, (void *) this->stereoBuf.hImageBuf , 0);
        }

        ~I420StereoFrame()
        {
            cudaFreeHost(this->stereoBuf.hImageBuf);
        }
};

class UYVYStereoFrame: public stereoFrame
{
    public:
        void convert(monoFrame* leftCamera, monoFrame* rightCamera)
        {
            this->leftCamera = leftCamera;
            this->rightCamera = rightCamera;

            cv::cuda::GpuMat dMergedFrame(cv::Size(this->node->g_config.stream_x_res, this->node->g_config.stream_y_res), CV_8UC2, stereoBuf.dImageBuf);
            
            cv::cuda::GpuMat dRGBAImageBufRight(cv::Size(this->node->g_config.mono_x_res, this->node->g_config.mono_y_res), CV_8UC2, rightCamera->frameBuf.dImageBuf);
            cv::cuda::GpuMat dRGBAImageBufLeft(cv::Size(this->node->g_config.mono_x_res, this->node->g_config.mono_y_res), CV_8UC2, leftCamera->frameBuf.dImageBuf);

            dRGBAImageBufRight.copyTo(dMergedFrame.operator()(cv::Rect(0, 0, dRGBAImageBufRight.cols, dRGBAImageBufRight.rows)));
            dRGBAImageBufLeft.copyTo(dMergedFrame.operator()(cv::Rect(dRGBAImageBufRight.cols, 0, dRGBAImageBufLeft.cols, dRGBAImageBufLeft.rows)));
            
            auto c_timedif = std::chrono::duration_cast<std::chrono::microseconds>(leftCamera->frameBuf.timestamp - rightCamera->frameBuf.timestamp);
            this->timedif = c_timedif.count();
        }

        void alloc(std::shared_ptr<NSSC>& node)
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->stereoBuf.hImageBuf, this->node->g_config.stereo_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->stereoBuf.dImageBuf, (void *) this->stereoBuf.hImageBuf , 0);
        }

        ~UYVYStereoFrame()
        {
            cudaFreeHost(this->stereoBuf.hImageBuf);
        }
};

stereoFrame *stereoFrame::make_frame(NSSC_FRAME_TYPE type, std::shared_ptr<NSSC>& node)
{
    switch(type)
    {
        case NSSC_FRAME_RGBA:
            return new RGBAStereoFrame;
            break;
        case NSSC_FRAME_I420:
            return new I420StereoFrame;
            break;
        case NSSC_FRAME_UYVY:
            return new UYVYStereoFrame;
            break;
    }
}