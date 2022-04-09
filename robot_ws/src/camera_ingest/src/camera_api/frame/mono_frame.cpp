#include "mono_frame.h"
#include "node.h"
#include "frame_struct.h"

class RGBAFrame: public nssc::framestruct::MonoFrame
{
    public:
        void convert() override
        {
            std::cout << "convert" << std::endl;
            if (this->node->g_config.frame_config.send_type == NDI_SEND_TRIANGULATION) { return; }
            if (CUDA_FAILED(cudaConvertColor(this->rgb_buf.dImageBuf, IMAGE_RGB8, this->rgba_buf.dImageBuf, IMAGE_RGBA8,
                                             this->node->g_config.frame_config.mono_x_res, this->node->g_config.frame_config.mono_y_res)))
            {
                this->node->printWarning(this->msg_caller, "RGBA convertion failed");
            }
            this->converted = true;
        }

        void alloc(std::shared_ptr<nssc::ros::NSSC>& node, int id) override
        {
            this->node = node;
            
            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->rgba_buf.hImageBuf, this->node->g_config.frame_config.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->rgba_buf.dImageBuf, (void *) this->rgba_buf.hImageBuf , 0);
            this->rgba_buf.id = id;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->rgb_buf.hImageBuf, this->node->g_config.frame_config.rgb_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->rgb_buf.dImageBuf, (void *) this->rgb_buf.hImageBuf , 0);
            this->rgb_buf.id = id;

            std::cout << "alloc" << std::endl;
        }

        ~RGBAFrame()
        {
            cudaFreeHost(this->rgba_buf.hImageBuf);
            cudaFreeHost(this->rgb_buf.hImageBuf);
        }
};

class I420Frame: public nssc::framestruct::MonoFrame
{
    public:
        void convert() override
        {
            if (CUDA_FAILED(cudaConvertColor(this->rgb_buf.dImageBuf, IMAGE_RGB8, this->rgba_buf.dImageBuf, IMAGE_I420,
                                             this->node->g_config.frame_config.mono_x_res, this->node->g_config.frame_config.mono_y_res)))
            {
                this->node->printWarning(this->msg_caller, "I420 convertion failed");
            }
        }

        void alloc(std::shared_ptr<nssc::ros::NSSC> &node, int id) override
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->rgba_buf.hImageBuf, this->node->g_config.frame_config.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->rgba_buf.dImageBuf, (void *) this->rgba_buf.hImageBuf , 0);
        }

        ~I420Frame()
        {
            cudaFreeHost(this->rgba_buf.hImageBuf);
            cudaFreeHost(this->rgb_buf.hImageBuf);
        }
};

class UYVYFrame: public nssc::framestruct::MonoFrame
{
    public:
        void convert() override
        {
            NppiSize oSizeROI = {(int)this->node->g_config.frame_config.mono_x_res, (int)this->node->g_config.frame_config.mono_y_res};
            int ret = nppiRGBToYUV422_8u_C3C2R((uint8_t*)this->rgb_buf.dImageBuf, this->node->g_config.frame_config.mono_x_res * 3,
                                               (uint8_t*)rgba_buf.dImageBuf, this->node->g_config.frame_config.mono_x_res * 2, oSizeROI);
            if(ret != 0)
            {
                this->node->printWarning(this->msg_caller, "NPPI convertion failed");
            }
        }

        void alloc(std::shared_ptr<nssc::ros::NSSC> &node, int id) override
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->rgba_buf.hImageBuf, this->node->g_config.frame_config.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->rgba_buf.dImageBuf, (void *) this->rgba_buf.hImageBuf , 0);
        }

        ~UYVYFrame()
        {
            cudaFreeHost(this->rgba_buf.hImageBuf);
            cudaFreeHost(this->rgb_buf.hImageBuf);
        }
};

nssc::framestruct::MonoFrame *nssc::framestruct::MonoFrame::makeFrame(NSSC_FRAME_TYPE type)
{
    switch(type)
    {
        case NSSC_FRAME_RGBA:
            return new RGBAFrame;
        case NSSC_FRAME_I420:
            return new I420Frame;
        case NSSC_FRAME_UYVY:
            return new UYVYFrame;
    }
}

void nssc::framestruct::MonoFrame::setTimestamp()
{
    auto timestamp = std::chrono::high_resolution_clock::now();
    this->rgba_buf.timestamp = timestamp;
    this->rgb_buf.timestamp = timestamp;
}