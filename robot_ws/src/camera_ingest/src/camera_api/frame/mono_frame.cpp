#include "mono_frame.h"

class RGBAFrame: public monoFrame
{
    public:
        void convert(frameBuffer* rgbBuf) override
        {
            if (CUDA_FAILED(cudaConvertColor(rgbBuf->dImageBuf, IMAGE_RGB8, this->frameBuf.dImageBuf, IMAGE_RGBA8, this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res)))
            {
                this->node->printWarning(this->msgCaller, "RGBA convertion failed");
            }
        }

        void alloc(std::shared_ptr<NSSC>& node, int id) override
        {
            this->node = node;
            
            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->frameBuf.hImageBuf, this->node->g_config.frameConfig.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->frameBuf.dImageBuf, (void *) this->frameBuf.hImageBuf , 0);
            this->frameBuf.id = id;
        }

        ~RGBAFrame()
        {
            cudaFreeHost(this->frameBuf.hImageBuf);
        }
};

class I420Frame: public monoFrame
{
    public:
        void convert(frameBuffer* rgbBuf) override
        {
            if (CUDA_FAILED(cudaConvertColor(rgbBuf->dImageBuf, IMAGE_RGB8, this->frameBuf.dImageBuf, IMAGE_I420, this->node->g_config.frameConfig.mono_x_res, this->node->g_config.frameConfig.mono_y_res)))
            {
                this->node->printWarning(this->msgCaller, "I420 convertion failed");
            }
        }

        void alloc(std::shared_ptr<NSSC> &node, int id) override
        {
            this->node = node;

            cudaSetDeviceFlags(cudaDeviceMapHost);
            cudaHostAlloc((void **)&this->frameBuf.hImageBuf, this->node->g_config.frameConfig.mono_buf_size, cudaHostAllocMapped);
            cudaHostGetDevicePointer((void **)&this->frameBuf.dImageBuf, (void *) this->frameBuf.hImageBuf , 0);
        }

        ~I420Frame()
        {
            cudaFreeHost(this->frameBuf.hImageBuf);
        }
};

class UYVYFrame: public monoFrame
{
    public:
        void convert(frameBuffer* rgbBuf) override
        {
            NppiSize oSizeROI = {(int)this->node->g_config.frameConfig.mono_x_res, (int)this->node->g_config.frameConfig.mono_y_res};
            int ret = nppiRGBToYUV422_8u_C3C2R((uint8_t*)rgbBuf->dImageBuf, this->node->g_config.frameConfig.mono_x_res * 3, (uint8_t*)frameBuf.dImageBuf, this->node->g_config.frameConfig.mono_x_res * 2, oSizeROI);
            if(ret != 0)
            {
                this->node->printWarning(this->msgCaller, "NPPI convertion failed");
            }
        }

        void alloc(std::shared_ptr<NSSC> &node, int id) override
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