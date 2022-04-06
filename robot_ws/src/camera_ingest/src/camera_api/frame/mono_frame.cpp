#include "mono_frame.h"
#include "node.h"
#include "frame_struct.h"

class RGBAFrame: public nssc::framestruct::MonoFrame
{
    public:
        void convert(nssc::framestruct::FrameBuffer* rgbBuf) override
        {
            if (CUDA_FAILED(cudaConvertColor(rgbBuf->dImageBuf, IMAGE_RGB8, this->frame_buf.dImageBuf, IMAGE_RGBA8, this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res)))
            {
                this->node->printWarning(this->msg_caller, "RGBA convertion failed");
            }
        }

        void alloc(std::shared_ptr<nssc::ros::NSSC>& node, int id) override
        {
            this->node = node;
            
            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->frame_buf.hImageBuf, this->node->g_config.frameConfig.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->frame_buf.dImageBuf, (void *) this->frame_buf.hImageBuf , 0);
            this->frame_buf.id = id;
        }

        ~RGBAFrame()
        {
            cudaFreeHost(this->frame_buf.hImageBuf);
        }
};

class I420Frame: public nssc::framestruct::MonoFrame
{
    public:
        void convert(nssc::framestruct::FrameBuffer* rgbBuf) override
        {
            if (CUDA_FAILED(cudaConvertColor(rgbBuf->dImageBuf, IMAGE_RGB8, this->frame_buf.dImageBuf, IMAGE_I420, this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res)))
            {
                this->node->printWarning(this->msg_caller, "I420 convertion failed");
            }
        }

        void alloc(std::shared_ptr<nssc::ros::NSSC> &node, int id) override
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->frame_buf.hImageBuf, this->node->g_config.frameConfig.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->frame_buf.dImageBuf, (void *) this->frame_buf.hImageBuf , 0);
        }

        ~I420Frame()
        {
            cudaFreeHost(this->frame_buf.hImageBuf);
        }
};

class UYVYFrame: public nssc::framestruct::MonoFrame
{
    public:
        void convert(nssc::framestruct::FrameBuffer* rgbBuf) override
        {
            NppiSize oSizeROI = {(int)this->node->g_config.frameConfig.mono_x_res, (int)this->node->g_config.frameConfig.mono_y_res};
            int ret = nppiRGBToYUV422_8u_C3C2R((uint8_t*)rgbBuf->dImageBuf, this->node->g_config.frameConfig.mono_x_res * 3, (uint8_t*)frame_buf.dImageBuf, this->node->g_config.frameConfig.mono_x_res * 2, oSizeROI);
            if(ret != 0)
            {
                this->node->printWarning(this->msg_caller, "NPPI convertion failed");
            }
        }

        void alloc(std::shared_ptr<nssc::ros::NSSC> &node, int id) override
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->frame_buf.hImageBuf, this->node->g_config.frameConfig.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->frame_buf.dImageBuf, (void *) this->frame_buf.hImageBuf , 0);
        }

        ~UYVYFrame()
        {
            cudaFreeHost(this->frame_buf.hImageBuf);
        }
};

nssc::framestruct::MonoFrame *nssc::framestruct::MonoFrame::makeFrame(NSSC_FRAME_TYPE type)
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

void nssc::framestruct::MonoFrame::setTimestamp()
{
    auto timestamp = std::chrono::high_resolution_clock::now();
    this->frame_buf.timestamp = timestamp;
}