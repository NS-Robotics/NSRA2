#include "ingest.h"

Ingest::Ingest(std::shared_ptr<NSSC>& node) : NSSC_ERRORS(node)
{
    this->node = node;
    this->node->printInfo(this->msgCaller, "Ingest!");
}