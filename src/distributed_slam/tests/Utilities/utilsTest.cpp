//
// Created by csadmin on 7/6/19.
//

#include "../include/utilsTest.h"

int main(int argc, char *argv[]) {
    std::string rootPath{"/home/csadmin/MultiAgentSlam-ROS/src/distributed_slam/tests"};
//    int status = chdir(rootPath.c_str());
//    if (status == -1) {
//        std::cerr << __PRETTY_FUNCTION__ << std::endl;
//        std::cerr << "Error: failed to changed working directory." << std::endl;
//        std::cerr << "\tDir <" << rootPath << ">" << std::endl;
//        std::cerr << "Exiting..." << std::endl;
//    }
    Logger::getInstance("test-env", rootPath);
    test2to1Mapping();

    return 0;
}

void test2to1Mapping() {
    std::cout << "Running... 2-1 Mapping." << std::endl;
    static auto lowBound{-10L};
    static auto highBound{10L};
    static auto delta{std::abs(highBound - lowBound)};
    static auto filePath{std::string{"/logs/test-env/mapped_pts.txt"}};
    Equations::getInstance()->mapSparcity(0.125);

    std::cout << "File Path : " << filePath << "\n"
              << "Mapping pts..." << std::endl;

    std::stringstream msg{};
    for (auto x = lowBound; x <= highBound; x++) {
        for (auto y = lowBound; y <= highBound; y++) {
            msg <<", " << x << ", " << y << ", ";

            auto map_pt{Equations::getInstance()->szudzikMapping(x, y)}; // This may need to be a small number


            msg << map_pt.point << ", "
                << map_pt.bounds[0] << ", "
                << map_pt.bounds[1] << std::endl;
            Logger::getInstance()->logTo(filePath, msg.str());
            msg.str(std::string{});
        }
        progressBar((x + highBound) * delta, std::pow(delta, 2));
    }
    std::cout << "\nFinished Mapping." << std::endl;
}

void progressBar(const float &processed, const float &totalWork) {
    std::cout << std::fixed << std::setprecision(2)<< "\rCompleted : [ " << (processed / totalWork) * 100 << "%]";
    std::cout.flush();
}