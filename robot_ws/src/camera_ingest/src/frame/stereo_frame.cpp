#include "stereo_frame.h"

class RGBAStereoFrame: public stereoFrame
{
    public:
        void convert(monoFrame *leftCamera, monoFrame *rightCamera, bool concatenate, bool resize)
        {
            this->leftCamera = leftCamera;
            this->rightCamera = rightCamera;
            this->concatenated = concatenate;

            if(concatenate)
            {
                cv::cuda::GpuMat dMergedFrame(cv::Size(this->node->g_config.frameConfig.stereo_x_res, this->node->g_config.frameConfig.stereo_y_res), CV_8UC4, this->concatenateBuf.dImageBuf);

                cv::cuda::GpuMat dRGBAImageBufRight(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res), CV_8UC4, leftCamera->frameBuf.dImageBuf);
                cv::cuda::GpuMat dRGBAImageBufLeft(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res), CV_8UC4, rightCamera->frameBuf.dImageBuf);

                dRGBAImageBufRight.copyTo(dMergedFrame.operator()(cv::Rect(0, 0, dRGBAImageBufRight.cols, dRGBAImageBufRight.rows)));
                dRGBAImageBufLeft.copyTo(dMergedFrame.operator()(cv::Rect(dRGBAImageBufRight.cols, 0, dRGBAImageBufLeft.cols, dRGBAImageBufLeft.rows)));

                this->stereoBuf = &this->concatenateBuf;
            }

            if(resize && concatenate)
            {
                cv::cuda::GpuMat inputFrame(cv::Size(this->node->g_config.frameConfig.stereo_x_res, this->node->g_config.frameConfig.stereo_y_res), CV_8UC4, this->concatenateBuf.dImageBuf);
                cv::cuda::GpuMat resizedFrame(cv::Size(this->node->g_config.frameConfig.resize_x_res, this->node->g_config.frameConfig.resize_y_res), CV_8UC4, this->resizeBuf.dImageBuf);

                cv::cuda::resize(inputFrame, resizedFrame, cv::Size(this->node->g_config.frameConfig.resize_x_res, this->node->g_config.frameConfig.resize_y_res));

                this->stereoBuf = &this->resizeBuf;
            }
            else if (resize && !concatenate)
            {
                this->node->printWarning(this->msgCaller, "Invalid input configuration");
            }
            
            auto c_timedif = std::chrono::duration_cast<std::chrono::microseconds>(leftCamera->frameBuf.timestamp - rightCamera->frameBuf.timestamp);
            this->timedif = c_timedif.count();
            
            if(this->timedif > this->node->g_config.frameConfig.max_frame_time_diff)
            {
                this->node->printWarning(this->msgCaller, "Frame timestamp difference out of bounds: " + std::to_string(this->timedif) + " us");
            }
        }

        void alloc(std::shared_ptr<NSSC>& node)
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->concatenateBuf.hImageBuf, this->node->g_config.frameConfig.stereo_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->concatenateBuf.dImageBuf, (void *)this->concatenateBuf.hImageBuf, 0);

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->resizeBuf.hImageBuf, this->node->g_config.frameConfig.resize_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->resizeBuf.dImageBuf, (void *)this->resizeBuf.hImageBuf, 0);
        }

        ~RGBAStereoFrame()
        {
            cudaFreeHost(this->concatenateBuf.hImageBuf);
            cudaFreeHost(this->resizeBuf.hImageBuf);
        }
};
/*
class I420StereoFrame: public stereoFrame
{
    public:
        void convert(monoFrame *leftCamera, monoFrame *rightCamera, bool concatenate, bool resize)
        {
            this->leftCamera = leftCamera;
            this->rightCamera = rightCamera;

            cv::cuda::GpuMat dMergedFrame(cv::Size(this->node->g_config.frameConfig.stream_x_res, std::round(this->node->g_config.frameConfig.stream_y_res * 1.5)), CV_8UC1, stereoBuf.dImageBuf);

            cv::cuda::GpuMat dRGBAImageBufRight(cv::Size(this->node->g_config.frameConfig.mono_x_res, std::round(this->node->g_config.frameConfig.mono_y_res * 1.5)), CV_8UC1, rightCamera->frameBuf.dImageBuf);
            cv::cuda::GpuMat dRGBAImageBufLeft(cv::Size(this->node->g_config.frameConfig.mono_x_res, std::round(this->node->g_config.frameConfig.mono_y_res * 1.5)), CV_8UC1, leftCamera->frameBuf.dImageBuf);

            dRGBAImageBufRight.copyTo(dMergedFrame.operator()(cv::Rect(0, 0, dRGBAImageBufRight.cols, dRGBAImageBufRight.rows)));
            dRGBAImageBufLeft.copyTo(dMergedFrame.operator()(cv::Rect(dRGBAImageBufRight.cols, 0, dRGBAImageBufLeft.cols, dRGBAImageBufLeft.rows)));
            
            auto c_timedif = std::chrono::duration_cast<std::chrono::microseconds>(leftCamera->frameBuf.timestamp - rightCamera->frameBuf.timestamp);
            this->timedif = c_timedif.count();
        }

        void alloc(std::shared_ptr<NSSC>& node)
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->stereoBuf.hImageBuf, this->node->g_config.frameConfig.stereo_buf_size, cudaHostAllocMapped);
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
        void convert(monoFrame *leftCamera, monoFrame *rightCamera, bool concatenate, bool resize)
        {
            this->leftCamera = leftCamera;
            this->rightCamera = rightCamera;

            cv::cuda::GpuMat dMergedFrame(cv::Size(this->node->g_config.frameConfig.stream_x_res, this->node->g_config.frameConfig.stream_y_res), CV_8UC2, stereoBuf.dImageBuf);
            
            cv::cuda::GpuMat dRGBAImageBufRight(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res), CV_8UC2, rightCamera->frameBuf.dImageBuf);
            cv::cuda::GpuMat dRGBAImageBufLeft(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res), CV_8UC2, leftCamera->frameBuf.dImageBuf);

            dRGBAImageBufRight.copyTo(dMergedFrame.operator()(cv::Rect(0, 0, dRGBAImageBufRight.cols, dRGBAImageBufRight.rows)));
            dRGBAImageBufLeft.copyTo(dMergedFrame.operator()(cv::Rect(dRGBAImageBufRight.cols, 0, dRGBAImageBufLeft.cols, dRGBAImageBufLeft.rows)));
            
            auto c_timedif = std::chrono::duration_cast<std::chrono::microseconds>(leftCamera->frameBuf.timestamp - rightCamera->frameBuf.timestamp);
            this->timedif = c_timedif.count();
        }

        void alloc(std::shared_ptr<NSSC>& node)
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->stereoBuf.hImageBuf, this->node->g_config.frameConfig.stereo_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->stereoBuf.dImageBuf, (void *) this->stereoBuf.hImageBuf , 0);
        }

        ~UYVYStereoFrame()
        {
            cudaFreeHost(this->stereoBuf.hImageBuf);
        }
};
*/
stereoFrame *stereoFrame::make_frame(NSSC_FRAME_TYPE type)
{
    switch(type)
    {
        case NSSC_FRAME_RGBA:
            return new RGBAStereoFrame;
            break;
        /*
        case NSSC_FRAME_I420:
            return new I420StereoFrame;
            break;
        case NSSC_FRAME_UYVY:
            return new UYVYStereoFrame;
            break;
        */
    }
}