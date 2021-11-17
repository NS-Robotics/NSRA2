#include "mono_frame.h"

class RGBAFrame: public monoFrame
{
    public:
        void convert(frameBuffer* rgbBuf)
        {
            if(g_config.resize_frame)
            {
                cv::cuda::GpuMat inputFrame(cv::Size(g_config.cam_x_res, g_config.cam_y_res), CV_8UC3, rgbBuf->dImageBuf);
                cv::cuda::GpuMat resizedFrame(cv::Size(g_config.mono_x_res, g_config.mono_y_res), CV_8UC3, this->resizeBuf.dImageBuf);

                cv::cuda::resize(inputFrame, resizedFrame, cv::Size(g_config.mono_x_res, g_config.mono_y_res));

                this->inputBuf = &this->resizeBuf;
            } else
            {
                this->inputBuf = rgbBuf;
            }
            if( CUDA_FAILED(cudaConvertColor(this->inputBuf->dImageBuf, IMAGE_RGB8, this->frameBuf.dImageBuf, IMAGE_RGBA8, g_config.mono_x_res, g_config.mono_y_res)) )
            {
                std::cout << "RGBA convert failed" << std::endl;
            } else
            {
                //std::cout << "successfully converted" << std::endl;
            }
        }

        void alloc()
        {
            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->frameBuf.hImageBuf, g_config.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->frameBuf.dImageBuf, (void *) this->frameBuf.hImageBuf , 0);

            if(g_config.resize_frame)
            {
                cudaSetDeviceFlags(cudaDeviceMapHost);
                cudaHostAlloc((void **)&this->resizeBuf.hImageBuf, g_config.mono_x_res * g_config.mono_y_res * 3, cudaHostAllocMapped);
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
            if(g_config.resize_frame)
            {
                cv::cuda::GpuMat inputFrame(cv::Size(g_config.cam_x_res, g_config.cam_y_res), CV_8UC3, rgbBuf->dImageBuf);
                cv::cuda::GpuMat resizedFrame(cv::Size(g_config.mono_x_res, g_config.mono_y_res), CV_8UC3, this->resizeBuf.dImageBuf);

                cv::cuda::resize(inputFrame, resizedFrame, cv::Size(g_config.mono_x_res, g_config.mono_y_res));

                this->inputBuf = &this->resizeBuf;
            } else
            {
                this->inputBuf = rgbBuf;
            }
            if( CUDA_FAILED(cudaConvertColor(this->inputBuf->dImageBuf, IMAGE_RGB8, this->frameBuf.dImageBuf, IMAGE_I420, g_config.mono_x_res, g_config.mono_y_res)) )
            {
                std::cout << "I420 convert failed" << std::endl;
            } else
            {
                //std::cout << "successfully converted" << std::endl;
            }
        }

        void alloc()
        {
            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->frameBuf.hImageBuf, g_config.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->frameBuf.dImageBuf, (void *) this->frameBuf.hImageBuf , 0);

            if(g_config.resize_frame)
            {
                cudaSetDeviceFlags(cudaDeviceMapHost);
                cudaHostAlloc((void **)&this->resizeBuf.hImageBuf, g_config.mono_x_res * g_config.mono_y_res * 3, cudaHostAllocMapped);
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
            NppiSize oSizeROI = {(int)g_config.mono_x_res, (int)g_config.mono_y_res};
            int ret = nppiRGBToYUV422_8u_C3C2R((uint8_t*)rgbBuf->dImageBuf, g_config.mono_x_res * 3, (uint8_t*)frameBuf.dImageBuf, g_config.mono_x_res * 2, oSizeROI);
            if(ret != 0)
            {
                std::cout << "NPPI conversion failed: " << ret << std::endl;
            }
        }

        void alloc()
        {
            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->frameBuf.hImageBuf, g_config.mono_buf_size, cudaHostAllocMapped);
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