#include <iostream>
#include <string>
#include <vector>

#include <zipper/unzipper.h>

int main(int argc, char** argv) {

    zipper::Unzipper unzipper("zipfile.zip");
    std::vector<zipper::ZipEntry> entries = unzipper.entries();
    // unzipper.close();

    for (auto&& entry : entries) {
        std::cout << entry.name << std::endl;
    }


    std::vector<unsigned char> unzipped_entry;
    // zipper::Unzipper unzipper("zipfile.zip");
    unzipper.extractEntryToMemory("rock README.txt", unzipped_entry);
    unzipper.close();

    std::cout << "Unzipped entry: " << unzipped_entry.size() << std::endl;

    for (auto&& e : unzipped_entry)
        std::cout << e;

    return 0;
}
