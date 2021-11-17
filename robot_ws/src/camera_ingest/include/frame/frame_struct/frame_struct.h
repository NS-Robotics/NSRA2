#pragma once

#include <chrono>

struct frameBuffer
{
public:
    void *hImageBuf; 
    void *dImageBuf;
    std::chrono::time_point<std::chrono::system_clock> timestamp;

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