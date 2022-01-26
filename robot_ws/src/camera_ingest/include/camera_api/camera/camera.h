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

#define ACQ_BUFFER_NUM          1
#define ACQ_TRANSFER_SIZE       (64 * 1024)
#define ACQ_TRANSFER_NUMBER_URB 64

class Camera : public NSSC_ERRORS
{
public:
    Camera(std::shared_ptr<NSSC> &node, std::shared_ptr<CyclicBarrier> &cb);
    ~Camera();
    static NSSC_STATUS init();

    NSSC_STATUS loadCamera(char device_serial_number[]);
    NSSC_STATUS startAcquisition();
    NSSC_STATUS closeCamera();

    monoFrame*  getFrame();
    NSSC_STATUS returnBuf(monoFrame* frame);

    NSSC_STATUS setExposure(float exposure_time);
    NSSC_STATUS setGain(float gain);

    std::atomic<bool> stop_age_check{true};

protected:
    std::shared_ptr<NSSC> node;
    GX_DEV_HANDLE h_device = nullptr;

private:
    NSSC_STATUS _printDeviceInfo();

    void GXDQBufThreadNDI();
    bool __checkFrameAge();

    std::thread GXDQ_thread;

    std::shared_ptr<CyclicBarrier> cb;

    std::atomic<bool> stream_running{false};
    std::atomic<int>  n_filled{0};
    std::atomic<int>  n_empty{0};

    std::string     cam_serial;
    std::string     msg_caller;
    const int64_t   g_i64ColorFilter = GX_COLOR_FILTER_NONE;
    int64_t         g_nPayloadSize = 0;
    uint64_t        nBufferNum = ACQ_BUFFER_NUM;
    bool            bStreamTransferSize = false;
    bool            bStreamTransferNumberUrb = false;

    moodycamel::BlockingConcurrentQueue<monoFrame*> buf_filled;
    moodycamel::BlockingConcurrentQueue<monoFrame*> buf_empty;

    std::chrono::time_point<std::chrono::system_clock> last_trigger;

    bool is_closed = false;
};

#endif //NSSC_CAMERA_