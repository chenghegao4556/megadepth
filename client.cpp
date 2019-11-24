//
// Created by chenghe on 11/23/19.
//
#include <megadepth_client.h>
int main(int argc,char* argv[])
{
    ///! check input parameters
    if (argc == 6 || argc == 4)
    {
        int inference_height, inference_width;
        if (argc == 6)
        {

            std::istringstream ( argv[4] ) >> inference_height;
            std::istringstream ( argv[5] ) >> inference_width;


        } else
        {
            inference_height = 256,
            inference_width = 512;
        }
        MegaDepth::MegaDepthClient client(argv[1], argv[2], argv[3],
                                          inference_height, inference_width);
        client.Start();
    }
    else
    {
        std::cout<<"wrong parameters"<<std::endl;
        return 1;
    }


    return 0;
}