#include <iostream>
#include <string>
#include <vector>
#include <memory>

class Element
{
public:
    Element(double x) : x(x) {}

    ~Element() {
        std::cout << "Element " << x << "\t(" << ((flag)?"true":"false") << ") deconstructed." << std::endl;
    }

    bool flag = false;
    double x;
};

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

class MyObj
{
public:
    MyObj() {}

    std::vector<std::shared_ptr<const Element>> add(const std::vector<double>& things) {

        // Create a temporary vector of ptrs to const elements which we will pass back
        std::vector<std::shared_ptr<const Element>> private_elements_ptrs;

        for (int i=0; i<things.size(); i++) {
            // Create a shared pointer to a new element
            auto p = std::make_shared<Element>(things[i]);

            // Add this shared ptr to our internal memory, which can be mutated.
            elements_.emplace_back(p);

            // Also add to the temporary vector which is read-only
            private_elements_ptrs.emplace_back(p);
        }

        return private_elements_ptrs;
    }

    // ------------------------------------------------------------------------

    void run() {
        for (auto&& e : elements_) {
            e->flag = !e->flag;
        }
    }

    // ------------------------------------------------------------------------

    void remove(int idx) {
        if (idx >= 0 && idx < elements_.size())
            elements_.erase(elements_.begin() + idx);
    }

    // ------------------------------------------------------------------------

    void print() {
        std::cout << std::endl << std::endl;
        std::cout << "Element Count: " << elements_.size() << std::endl;
        for (auto&& e : elements_) {
            std::cout << "\t Element " << e->x << "\tis " << ((e->flag)?"true":"false") << std::endl;
        }
    }

private:
    std::vector<std::shared_ptr<Element>> elements_;
    
};

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

int main(int argc, char** argv) {

    MyObj obj;

    // Create some things and add them to the object
    std::vector<double> things {7,24,13,5,564,79,8};
    auto private_elements = obj.add(things);
    things.clear();

    // What does it look like?
    obj.print();

    // Flip the flags of the elements
    obj.run();

    // What does it look like?
    obj.print();


    // Won't compile
    // private_elements[0]->flag = false;


    // Read out the private element ptrs
    std::cout << std::endl << std::endl;
    std::cout << "Private Element Count: " << private_elements.size() << std::endl;
    for (auto&& e : private_elements) {
        std::cout << "\t Private Element " << e->x << "\tis " << ((e->flag)?"true":"false") << std::endl;
    }


    // Remove an element from the internal state
    obj.remove(2);


    // Read out the private element ptrs -- the element still exists! 
    std::cout << std::endl << std::endl;
    std::cout << "Private Element Count: " << private_elements.size() << std::endl;
    for (auto&& e : private_elements) {
        std::cout << "\t Private Element " << e->x << "\tis " << ((e->flag)?"true":"false") << std::endl;
    }

    // But not here
    obj.print();

    return 0;
}
