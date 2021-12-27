#include "mono_frame.h"

class RGBAFrame: public monoFrame
{
    public:
        void convert(frameBuffer* rgbBuf)
        {
            if(this->node->g_config.frameConfig.resize_frame)
            {
                auto start0 = std::chrono::high_resolution_clock::now();
                cv::cuda::GpuMat inputFrame(cv::Size(this->node->g_config.frameConfig.cam_x_res, this->node->g_config.frameConfig.cam_y_res), CV_8UC3, rgbBuf->dImageBuf);
                cv::cuda::GpuMat resizedFrame(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res), CV_8UC3, this->resizeBuf.dImageBuf);

                cv::cuda::resize(inputFrame, resizedFrame, cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res));

                this->inputBuf = &this->resizeBuf;
                auto stop0 = std::chrono::high_resolution_clock::now();
                auto duration0 = std::chrono::duration_cast<std::chrono::microseconds>(stop0 - start0);
                //this->node->printInfo(this->msgCaller, "Frame timing: resize - " + std::to_string(duration0.count()));
            } else
            {
                this->inputBuf = rgbBuf;
            }

            cv::Mat sendFrame(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res), CV_8UC3, this->inputBuf->hImageBuf);

            auto end = std::chrono::system_clock::now();
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);

            cv::putText(sendFrame, std::ctime(&end_time), cv::Point(10, sendFrame.rows / 2 + 100), //top-left position
                        cv::FONT_HERSHEY_DUPLEX,
                        1.0,
                        cv::Scalar(254, 0, 0), //font color
                        2);

            cv::cuda::GpuMat inputFrame(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res), CV_8UC3, this->inputBuf->dImageBuf);
            cv::cuda::cvtColor(this->inputBuf->dImageBuf, this->frameBuf.dImageBuf, cv::COLOR_RGB2RGBA);

            /*
            if( CUDA_FAILED(cudaConvertColor(this->inputBuf->dImageBuf, IMAGE_RGB8, this->frameBuf.dImageBuf, IMAGE_RGBA8, this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res)) )
            {
                std::cout << "RGBA convert failed" << std::endl;
            } else
            {
                
            }
            */
        }

        void alloc(std::shared_ptr<NSSC>& node)
        {
            this->node = node;
            
            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->frameBuf.hImageBuf, this->node->g_config.frameConfig.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->frameBuf.dImageBuf, (void *) this->frameBuf.hImageBuf , 0);

            if(this->node->g_config.frameConfig.resize_frame)
            {
                cudaSetDeviceFlags(cudaDeviceMapHost);
                cudaHostAlloc((void **)&this->resizeBuf.hImageBuf, this->node->g_config.frameConfig.mono_x_res * this->node->g_config.frameConfig.mono_y_res * 3, cudaHostAllocMapped);
                cudaHostGetDevicePointer((void **)&this->resizeBuf.dImageBuf, (void *) this->resizeBuf.hImageBuf , 0);
            }
        }

        ~RGBAFrame()
        {
            cudaFreeHost(this->frameBuf.hImageBuf);
            cudaFreeHost(this->resizeBuf.hImageBuf);
        }
};

class I420Frame: public monoFrame
{
    public:
        void convert(frameBuffer* rgbBuf)
        {
            if(this->node->g_config.frameConfig.resize_frame)
            {
                cv::cuda::GpuMat inputFrame(cv::Size(this->node->g_config.frameConfig.cam_x_res, this->node->g_config.frameConfig.cam_y_res), CV_8UC3, rgbBuf->dImageBuf);
                cv::cuda::GpuMat resizedFrame(cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res), CV_8UC3, this->resizeBuf.dImageBuf);

                cv::cuda::resize(inputFrame, resizedFrame, cv::Size(this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res));

                this->inputBuf = &this->resizeBuf;
            } else
            {
                this->inputBuf = rgbBuf;
            }
            if( CUDA_FAILED(cudaConvertColor(this->inputBuf->dImageBuf, IMAGE_RGB8, this->frameBuf.dImageBuf, IMAGE_I420, this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res)) )
            {
                std::cout << "I420 convert failed" << std::endl;
            } else
            {
                //std::cout << "successfully converted" << std::endl;
            }
        }

        void alloc(std::shared_ptr<NSSC>& node)
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->frameBuf.hImageBuf, this->node->g_config.frameConfig.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->frameBuf.dImageBuf, (void *) this->frameBuf.hImageBuf , 0);

            if(this->node->g_config.frameConfig.resize_frame)
            {
                cudaSetDeviceFlags(cudaDeviceMapHost);
                cudaHostAlloc((void **)&this->resizeBuf.hImageBuf, this->node->g_config.frameConfig.mono_x_res * this->node->g_config.frameConfig.mono_y_res * 3, cudaHostAllocMapped);
                cudaHostGetDevicePointer((void **)&this->resizeBuf.dImageBuf, (void *) this->resizeBuf.hImageBuf , 0);
            }
        }

        ~I420Frame()
        {
            cudaFreeHost(this->frameBuf.hImageBuf);
            cudaFreeHost(this->resizeBuf.hImageBuf);
        }
};

class UYVYFrame: public monoFrame
{
    public:
        void convert(frameBuffer* rgbBuf)
        {
            NppiSize oSizeROI = {(int)this->node->g_config.frameConfig.mono_x_res, (int)this->node->g_config.frameConfig.mono_y_res};
            int ret = nppiRGBToYUV422_8u_C3C2R((uint8_t*)rgbBuf->dImageBuf, this->node->g_config.frameConfig.mono_x_res * 3, (uint8_t*)frameBuf.dImageBuf, this->node->g_config.frameConfig.mono_x_res * 2, oSizeROI);
            if(ret != 0)
            {
                std::cout << "NPPI conversion failed: " << ret << std::endl;
            }
        }

        void alloc(std::shared_ptr<NSSC>& node)
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->frameBuf.hImageBuf, this->node->g_config.frameConfig.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->frameBuf.dImageBuf, (void *) this->frameBuf.hImageBuf , 0);
        }

        ~UYVYFrame()
        {
            cudaFreeHost(this->frameBuf.hImageBuf);
        }
};

monoFrame *monoFrame::make_frame(NSSC_FRAME_TYPE type)
{
    switch(type)
    {
        case NSSC_FRAME_RGBA:
            return new RGBAFrame;
            break;
        case NSSC_FRAME_I420:
            return new I420Frame;
            break;
        case NSSC_FRAME_UYVY:
            return new UYVYFrame;
            break;
    }
}

void monoFrame::setTimestamp()
{
    auto timestamp = std::chrono::high_resolution_clock::now();
    this->frameBuf.timestamp = timestamp;
}