#ifndef NSSC_CAMERA_
#define NSSC_CAMERA_

#include "GxIAPI.h"
#include "DxImageProc.h"
#include "node.h"
#include "nssc_errors.h"
#include "blockingconcurrentqueue.h"
#include "mono_frame.h"
#include "cam_sync.h"
#include "config.h"

#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <cuda_runtime_api.h>
#include <jetson-utils/cudaColorspace.h>
#include <jetson-utils/cudaMappedMemory.h>
#include <cuda.h>

#define ACQ_BUFFER_NUM          5           
#define ACQ_TRANSFER_SIZE       (64 * 1024)    
#define ACQ_TRANSFER_NUMBER_URB 64             

class Camera : public NSSC_ERRORS
{
public:
    Camera(std::shared_ptr<NSSC> &node, std::shared_ptr<CyclicBarrier> &cb);
    ~Camera();
    NSSC_STATUS init();

    NSSC_STATUS LoadCamera(char device_serial_number[]);
    NSSC_STATUS startAcquisition();
    NSSC_STATUS CloseCamera();

    NSSC_STATUS getEmStatus();

    monoFrame* getFrame();
    NSSC_STATUS returnBuf(monoFrame* frame);

protected:
    std::shared_ptr<NSSC> node;
    GX_DEV_HANDLE hDevice = NULL;

private:
    NSSC_STATUS _PrintDeviceInfo();

    globalConfig g_config;

    void GXDQBufThread();
    void GXDQBufThreadNDI();

    std::thread GXDQThread;
    std::thread GXDQThreadNDI;

    std::shared_ptr<CyclicBarrier> cb;

    std::atomic<bool> streamON{false};
    std::atomic<int>  numOfFilled{0};
    std::atomic<int>  numOfEmpty{0};

    std::string camSerial;
    std::string msgCaller;
    int64_t     g_i64ColorFilter = GX_COLOR_FILTER_NONE;
    int64_t     g_nPayloadSize = 0;
    uint64_t    nBufferNum = ACQ_BUFFER_NUM;
    bool        bStreamTransferSize = false;
    bool        bStreamTransferNumberUrb = false;
    bool        camera_loaded = false;

    moodycamel::BlockingConcurrentQueue<monoFrame*> filledFrameBuf;
    moodycamel::BlockingConcurrentQueue<monoFrame*> emptyFrameBuf;
};

#endif //NSSC_CAMERA_