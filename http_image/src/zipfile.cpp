#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <future>

#include <cpr/cpr.h>

int main(int argc, char** argv) {
    const std::string tileserver = "http://terrain.party/api/export?name=rock&box=-111.633553,40.268464,-111.637865,40.266012";

    std::vector<std::future<void>> futures;

    {
        auto future = cpr::GetCallback([](const cpr::Response& r) {
            std::fstream imgout("files.zip", std::ios::out | std::ios::binary);
            imgout.write(r.text.c_str(), r.text.size());
            imgout.close();
            std::cout << "Hello from the async past" << std::endl;
        }, cpr::Url{tileserver});

        // Note how if this line were commented out (i.e., `future` went out of scope here)
        // then the async call would become blocking -- you'll see "past" before "present"
        futures.push_back(std::move(future));
    }

    std::cout << "Hello from the present" << std::endl;
}
