#include "cli.h"

void CLI::CLIFunc()
{
    char *buf;

    while ((buf = readline("\033[1;32m[NSSC client] >> \033[0m")) != nullptr && this->cliON.load())
    {
        if (strlen(buf) > 0)
        {
            add_history(buf);
        }
        else
        {
            continue;
        }

        std::vector<char *> cmd = {};

        if (__procArg(buf, cmd) != NSSC_STATUS_SUCCESS)
        {
            this->printError("Bad argument!");
            continue;
        }

        if (strcmp(cmd[0], "NDI") == 0)
        {
            rawNDI();
        }
        else if (strcmp(cmd[0], "ingest") == 0)
        {
            int ingestAmount;
            if (getIntArg(cmd, 'n', ingestAmount) != NSSC_STATUS_SUCCESS || ingestAmount < 1)
            {
                this->printError("Bad argument!");
                continue;
            }
            char *setName;
            if (getStrArg(cmd, 'd', &setName) != NSSC_STATUS_SUCCESS)
            {
                this->printError("Bad argument!");
                continue;
            }
            this->node->g_config.ingestConfig.set_name = setName;
            this->node->g_config.ingestConfig.ingest_amount = ingestAmount;
            run_ingest();
            delete[] setName;
        }
        else if (strcmp(cmd[0], "calibrate") == 0)
        {
            char *setName;
            if (getStrArg(cmd, 'd', &setName) != NSSC_STATUS_SUCCESS)
            {
                this->printError("Bad argument!");
            }
            else
            {
                run_calibration(setName);
                delete[] setName;
            }
        }
        else if (strcmp(cmd[0], "cancel") == 0)
        {
            cancel();
        }
        else if (strcmp(cmd[0], "exit") == 0)
        {
            this->cliON = false;
            exit();
            break;
        }
        else if (strcmp(cmd[0], "help") == 0)
        {
            printf("\033[1;34m[Executor] \033[0mUsage:\n"
                   "  NDI                                       - Toggle raw NDI stream\n"
                   "  ingest [-n number of images -d set name]  - Start the calibration capture\n"
                   "  calibrate [-d image set name]             - Run the calibration\n"
                   "  run [-c calibration config]               - Run NSSC\n"
                   "  exit                                      - Close the application\n");
        }
        else
        {
            this->printError("Unknown command!");
        }

        free(buf);
    }
}

void CLI::printError(const char *message)
{
    printf("\033[1;34m[Executor] \033[1;31mError: %s\033[0m\n", message);
}

void CLI::openCLI(std::shared_ptr<NSSC> &node)
{
    this->node = node;
    this->cliON = true;
    this->CLIThread = std::thread(&CLI::CLIFunc, this);
}

void CLI::closeCLI()
{
    this->cliON = false;
    this->CLIThread.join();
    //this->node->printInfo("CLI", "CLI closed!");
}