#include <iostream>
#include <string>
#include <fstream>

#include <cpr/cpr.h>

int main(int argc, char** argv) {
    const std::string tileserver = "http://mt1.google.com/vt/lyrs=s&x=0&y=0&z=0";
    auto r = cpr::Get(cpr::Url{tileserver});


    std::cout << r.url << std::endl; // http://mt1.google.com/vt/lyrs=s&x=0&y=0&z=0
    std::cout << r.status_code << std::endl; // 200
    std::cout << r.header["content-type"] << std::endl; // image/jpeg
    // std::cout << r.text << std::endl;
    std::cout << r.text.size() << std::endl;

    // Save the response text as a binary
    std::fstream imgout("myimg.jpg", std::ios::out | std::ios::binary);
    imgout.write(r.text.c_str(), r.text.size());
    imgout.close();
}
