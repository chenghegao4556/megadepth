//
// Created by chenghe on 11/23/19.
//
#include <megadepth_client.h>
int main(int argc,char* argv[])
{
    ///! check input parameters
    if (argc != 4)
    {
        std::cout<<"wrong parameters"<<std::endl;
        return 1;
    }
    MegaDepth::MegaDepthClient client(argv[1], argv[2], argv[3]);
    client.Start();

    return 0;
}