/**
 * Producer-Comsumer example, written in C++ May 4, 2014
 * Compiled on OSX 10.9, using:
 * g++ -std=c++11 producer_consumer.cpp
 **/

#include <iostream>           
#include <thread>             
#include <mutex>              
#include <condition_variable>
#include <chrono>

std::mutex mtx;
std::condition_variable cv;

int meal = 0;

/* Consumer */
void waiter(int ordernumber) {
  std::unique_lock<std::mutex> lck(mtx);
  cv.wait(lck, [](){ return meal != 0; });
  std::cout << "Order: ";
  std::cout << ordernumber + 1 << " being taken care of with ";
  std::cout << meal - 1 << " meals also ready." << std::endl;
  meal--;
}

/* Producer */
void makeMeal(int ordernumber) {
  std::unique_lock<std::mutex> lck(mtx);
  meal++;
  std::cout << "Chef: Made meal " << meal << std::endl;
  cv.notify_one();
}

int main() {

  std::thread chefs[10];
  std::thread waiters[10];

  /* Initialize customers and cheifs */
  for (int order = 0; order < 10; order++) {
    chefs[order] = std::thread(makeMeal, order);
    waiters[order] = std::thread(waiter, order);
  }

  // for (int order = 0; order < 10; order++) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(15));
  //   waiters[order] = std::thread(waiter, order);
  // }

  /* Join the threads to the main threads */
  for (int order = 0; order < 10; order++) {
    waiters[order].join();   
    chefs[order].join(); 
  }

  return 0;
}