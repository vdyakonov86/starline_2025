#include "PcdLoader.hpp"
#include "PcdVisualizer.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_pcd>" << std::endl;
        return -1;
    }

    PcdLoader loader;
    if (!loader.load(argv[1]))
    {
        return -1;
    }

    auto cloud = loader.get();

    PcdVisualizer visualizer;

    // пример: переключение режима
    // std::cout << "Displaying default color..." << std::endl;
    // visualizer.showCloud(cloud, DisplayMode::Default);
    // visualizer.spin();

    std::cout << "Displaying by intensity..." << std::endl;
    visualizer.showCloud(cloud, DisplayMode::Intensity);
    visualizer.spin();

    // std::cout << "Displaying by height..." << std::endl;
    // visualizer.showCloud(cloud, DisplayMode::Height);
    // visualizer.spin();

    return 0;
}
