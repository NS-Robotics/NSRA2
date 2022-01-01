#ifndef NSSC_INGEST_
#define NSSC_INGEST_

#include "nssc_errors.h"
#include "camera_manager.h"
#include "node.h"

#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"
#include "rapidxml_print.hpp"

#include <opencv2/core/core.hpp>

#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>

class Ingest : public NSSC_ERRORS
{
    public:
        Ingest(std::shared_ptr<NSSC> &node, std::shared_ptr<cameraManager>& camManager);
        void cancelIngest();

    private:
        std::shared_ptr<NSSC> node;
        std::shared_ptr<cameraManager>  camManager;

        std::string msgCaller = "Ingest";

        std::string setPath;

        void ingestThread();
        std::thread iThread;
        std::atomic<bool> runIngest{false};

        void saveConfig();
        void editConfig();
};

#endif  //NSSC_INGEST_