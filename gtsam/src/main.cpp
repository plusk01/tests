#include <iostream>
#include <functional>

#include <gtsam/nonlinear>

#include <gtsam/geometry/Pose2.h>


int main(int argc, char const *argv[]) {

    Test t1;

    t1.square(5);
    std::cout << "t1 count: " << t1.get_count() << std::endl;

    t1.square(7);
    std::cout << "t1 count: " << t1.get_count() << std::endl;

    func(std::bind(&Test::square, &t1, std::placeholders::_1));
    std::cout << "t1 count: " << t1.get_count() << std::endl;

    return 0;
}