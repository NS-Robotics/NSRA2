#include "cli.h"

CLI::CLI()
{
    this->streamON = true;
    this->CLIThread = std::thread(&CLI::CLIFunc, this);
}

void CLI::CLIFunc()
{
    char *buf;
    while ((buf = readline(">> ")) != nullptr && this->streamON.load())
    {
        if (strlen(buf) > 0)
        {
            add_history(buf);
        }

        printf("[%s]\n", buf);

        // readline malloc's a new buffer every time.
        free(buf);
    }
}

void CLI::stopCLI()
{
    this->streamON = false;
}