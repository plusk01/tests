#include <iostream>
#include <string>
#include <fstream>

#include <cpr/cpr.h>

int main(int argc, char** argv) {
    const std::string tileserver = "https://static.pexels.com/photos/2422/sky-earth-galaxy-universe.jpg";

    auto future = cpr::GetCallback([](const cpr::Response& r) {
        std::fstream imgout("myasyncimg.jpg", std::ios::out | std::ios::binary);
        imgout.write(r.text.c_str(), r.text.size());
        imgout.close();
        std::cout << "Hello from the async past" << std::endl;
    }, cpr::Url{tileserver});

    std::cout << "Hello from the present" << std::endl;
}
