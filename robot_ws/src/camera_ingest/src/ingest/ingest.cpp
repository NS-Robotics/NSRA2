#include "ingest.h"

Ingest::Ingest(std::shared_ptr<NSSC>& node, int ingestAmount) : NSSC_ERRORS(node)
{
    this->node = node;
    this->node->printInfo(this->msgCaller, "Ingest!");
    this->ingestAmount = ingestAmount;

    this->runIngest = true;
    this->iThread = std::thread(&Ingest::runIngest, this);
}

void Ingest::ingestThread()
{
    
}