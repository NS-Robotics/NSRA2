#ifndef NSSC_FRAME_FRAME_STRUCT_
#define NSSC_FRAME_FRAME_STRUCT_

#include <chrono>

struct frameBuffer
{
public:
    void *hImageBuf; 
    void *dImageBuf;
    std::chrono::time_point<std::chrono::system_clock> timestamp;
    int id;

    frameBuffer()
    {
    }

    frameBuffer(void *hBuf, void *dBuf, std::chrono::time_point<std::chrono::system_clock> frameTimestamp)
    {
        hImageBuf = (void *)hBuf;
        dImageBuf = (void *)dBuf;
        timestamp = frameTimestamp;
    }

    frameBuffer(frameBuffer *buf)
    {
        hImageBuf = (void *)buf->hImageBuf;
        dImageBuf = (void *)buf->dImageBuf;
        timestamp = buf->timestamp;
    }
};

#endif //NSSC_FRAME_FRAME_STRUCT_