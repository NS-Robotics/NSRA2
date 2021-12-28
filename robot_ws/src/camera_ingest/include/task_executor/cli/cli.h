#ifndef NSSC_TASK_EXECUTOR_CLI_
#define NSSC_TASK_EXECUTOR_CLI_

#include <readline/readline.h>
#include <readline/history.h>

#include <thread>
#include <atomic>
#include <string.h>
#include <boost/lexical_cast.hpp>

#include "node.h"
#include "nssc_errors.h"

class CLI
{
public:
    void CLIFunc();
    void openCLI(std::shared_ptr<NSSC> &node);
    void closeCLI();

    virtual void exit() = 0;
    virtual void rawNDI() = 0;
    virtual void run_ingest() = 0;
    virtual void run_calibration(char *setName) = 0;
    virtual void cancel() = 0;

    std::atomic<bool> cliON{false};

private:
    std::thread CLIThread;

    std::shared_ptr<NSSC> node;

    void printError(const char *message);

    NSSC_STATUS procArg(char *buf, std::vector<char *> &cmd);
    NSSC_STATUS getIntArg(std::vector<char *> cmd, char par, int &ret);
    NSSC_STATUS getBoolArg(std::vector<char *> cmd, char par, bool &ret);
    NSSC_STATUS getStrArg(std::vector<char *> cmd, char par, char **ret);
    const char del[2] = "-";
    const char arg_del[2] = " ";
};

#endif // NSSC_TASK_EXECUTOR_CLI_