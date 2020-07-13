#include <iostream>
#include <memory>

class Base
{
public:
  virtual void update() = 0;
  virtual ~Base() = default;

  virtual void tick() = 0;
  
};

// ----------------------------------------------------------------------------

class A : public Base
{
public:
  A() = default;
  ~A() = default;
  
  virtual void update() override
  {
    std::cout << "update from A" << std::endl;
  }

  virtual void tick() override
  {
    std::cout << "tick from A" << std::endl;

    update();
  }

};

// ----------------------------------------------------------------------------

class B : public A
{
public:
  B() = default;
  ~B() = default;

  virtual void update() override
  {
    A::update();

    std::cout << "update from B" << std::endl;
    impl_details();
  }

private:
  void impl_details()
  {
    std::cout << "implementation details of B" << std::endl;
  }
  
};

// ----------------------------------------------------------------------------

int main(int argc, char const *argv[])
{

  std::unique_ptr<Base> b;
  b.reset(new B());

  b->update();
  std::cout << std::endl << std::endl;
  b->tick();

  return 0;
}