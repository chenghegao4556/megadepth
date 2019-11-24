//
// Created by chenghe on 11/23/19.
//
#include "cxxopts.hpp"
#include <megadepth_client.h>

cxxopts::ParseResult
parse(int argc, char* argv[])
{
    try
    {
        cxxopts::Options options(argv[0], " - MegaDepth Client Options");
        options
                .positional_help("[optional args]")
                .show_positional_help();

        options
                .allow_unrecognised_options()
                .add_options()
                        ("n, network_path", "MegaDepth weight path",
                                cxxopts::value<std::string>()->default_value("../data/mega_depth_net.pt"))
                        ("v, video",  "test video path",
                                cxxopts::value<std::string>()->default_value("../data/kitti.mp4"))
                        ("y, yaml",   "camera intrinsic file",
                                cxxopts::value<std::string>()->default_value("../data/kitti.yml"))
                        ("h, height", "inference height", cxxopts::value<int>()->default_value("256"))
                        ("w, width",  "inference width",  cxxopts::value<int>()->default_value("512"))
                        ("help", "Print help");

        auto result = options.parse(argc, argv);

        if (result.count("help"))
        {
            std::cout << options.help() << std::endl;
            exit(0);
        }
        std::cout<<"**weight path :"<<result["n"].as<std::string>()<<std::endl;
        std::cout<<"**video  path :"<<result["v"].as<std::string>()<<std::endl;
        std::cout<<"**yaml   path :"<<result["y"].as<std::string>()<<std::endl;
        std::cout<<"**inference height :"<<result["h"].as<int>()<<std::endl;
        std::cout<<"**inference width  :"<<result["w"].as<int>()<<std::endl;


        return result;

    } catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }
}

int main(int argc,char* argv[])
{

   auto result = parse(argc, argv);
   MegaDepth::MegaDepthClient client(result["n"].as<std::string>(),
                                     result["v"].as<std::string>(),
                                     result["y"].as<std::string>(),
                                     result["h"].as<int>(),
                                     result["w"].as<int>());
   client.Start();


    return 0;
}