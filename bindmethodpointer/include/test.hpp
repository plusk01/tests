#pragma once

#include <iostream>

class Test
{
public:
    Test() {};

    void square(int a) {
        std::cout << "a squared: " << a*a << std::endl;
        count++;
    }

    int get_count() {
        return count;
    }

private:
    int count = 0;
    
};