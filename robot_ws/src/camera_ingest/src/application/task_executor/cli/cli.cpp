#include "cli.h"

void CLI::CLIFunc()
{
    char *buf;

    while ((buf = readline("\033[1;32m[NSSC client]\033[0m >> ")) != nullptr && this->cliON.load())
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
            if(cmd.size() == 1)
            {
                toggleNDI(this->node->g_config.frameConfig.mono_stream);
                continue;
            }
            if(cmd.size() == 2)
            {
                bool raw_stream;
                if (getBoolArg(cmd, 'r', raw_stream) != NSSC_STATUS_SUCCESS)
                {
                    this->printError("Bad argument!");
                    continue;
                }
                _toggleNDIsource(NDI_SEND_RAW);
            } else
            {
                bool mono_stream;
                if (getBoolArg(cmd, 'm', mono_stream) != NSSC_STATUS_SUCCESS)
                {
                    this->printError("Bad argument!");
                    continue;
                }
                toggleNDI(false);
                bool raw_stream;
                if (getBoolArg(cmd, 'r', raw_stream) != NSSC_STATUS_SUCCESS)
                {
                    this->printError("Bad argument!");
                    continue;
                }
                _toggleNDIsource(NDI_SEND_RAW);
            }
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
        else if (strcmp(cmd[0], "triangulate") == 0)
        {
            if (cmd.size() == 1)
            {
                run_triangulation(const_cast<char *>(this->node->g_config.triangulationConfig.standard_config_file));
                continue;
            }
            char *setName;
            if (getStrArg(cmd, 'd', &setName) != NSSC_STATUS_SUCCESS)
            {
                this->printError("Bad argument!");
            }
            else
            {
                run_triangulation(setName);
                delete[] setName;
            }
        }
        else if (strcmp(cmd[0], "detect") == 0)
        {
            run_detection();
        }
        else if (strcmp(cmd[0], "calib_origin") == 0)
        {
            find_triangulation_origin();
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
                   "  triangulate [-d calibration config]       - Prepare NSSC for object detection\n"
                   "  calib_origin                              - Recalibrate the robot origin\n"
                   "  detect                                    - Run the NSSC object detection\n"
                   "  cancel                                    - Cancel the currently running process\n"
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
}