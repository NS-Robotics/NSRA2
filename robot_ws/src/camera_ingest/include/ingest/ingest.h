#ifndef NSSC_INGEST_
#define NSSC_INGEST_

#include "nssc_errors.h"
#include "camera_manager.h"
#include "node.h"

class Ingest : public NSSC_ERRORS
{
    public:
        Ingest(std::shared_ptr<NSSC> &node);

    private:
        std::shared_ptr<NSSC> node;

        std::string msgCaller = "Ingest";
};

#endif  //NSSC_INGEST_