Move Semantics
==============

Essentially, move semantics allows the swaping of the underlying data member memory between two objects (think `std::swap()`). This limits the amount of copying needed and comes from the fact that one of the objects involved in the swap was temporary. Rvalue references (e.g., `int&&`) were introduced in C++11 to allow the compiler to know when something was temporary.

---

In C++ (all versions), **Copy Semantics** refer to the following scenarios where copies are made:
    1. When setting one already initialized object to another (`a1 = a2`). (**Assignment operator**)
    2. When creating a new object from an existing object (`B b1 = b2` or `B b1(b2)`). (**Copy constructor**)

Move semantics started in C++11 and describes the ability of the compiler to minimize the number of copy operations.  For example, consider the following code:

```c++
std::string foo();

std::string x = "Parker";
std::string y = "Lusk";

std::string a(x);
std::string b(x + " " + y);
std::string c(foo());
```

Note that only the creation of string `a` are copy semantics required. Strings `b` and `c` are created from temporary, intermediate values (called `rvalues`) and could be made more efficient by enabling move semantics.

This motivates the desired to endow the C++ compiler with the ability to determine whether or not a value is temporary or not. This gets into the discussion of `lvalues` (locater values, have a memory address) vs `rvalues` (not `lvalues` -- i.e., temporary).

```c++
int a = 11;     // ok, a is a lvalue, 11 is an rvalue
int b = a;      // ok, b is a copy of a (different memory)
int *p1 = &a;   // ok, p1 and a are lvalues
p1 = &b;        // ok, pointers are mutable

int c = a + 2;  // ok, c is an lvalue, a+2 is an rvalue

int foo() { return 5 };
foo() = 1;      // error, foo() is an rvalue
int d = foo();  // ok, foo() is an rvalue

int& foobar() { static int i; return i; }
foo() = 1;      // ok, foo() is an lvalue -- and now the static int i == 1
int e = foo();  // ok, foo() is an lvalue
```

Once `rvalues` are understood, then comes another C++11 feature known as `rvalue references`. Note that the typical term `reference` which has existed since the beginning of C++ is more precisely called `lvalue reference`.

```c++
// lvalue reference
int a = 1;
int& z = a;
assert(z == 1 && a == 1);
z++;
assert(z == 2 && a == 2);

int& y = 2; // error, can't create a reference to an rvalue

// const lvalue reference
int a = 1;
const int& y = 2; // ok, because it's non-mutable

// rvalue reference
int a = 1;
int&& z = a;    // error, can't bind an lvalue to an rvalue reference
int&& z = a+5;  // ok, a+5 is an rvalue (could have done `int z = a+5`
                //  but note that the real use for rvalue reference is
                //  function overloading for move semantics -- this
                //  allows the compiler to say, "hey this is a
                //    temporary value, let's use a different function
                //    than the typical copy function"

// const rvalue reference
int a = 1;
const int&& z = a+1;    // I mean, it's fine but it doesn't make a lot of sense
                        // because by using an rvalue reference you're saying
                        // you want to mutate a temporary value...
```

This allows move semantics. Also note that before C++11 it was the *Rule of Three*, but now it is the *Rule of Five*, see [here](https://en.wikipedia.org/wiki/Rule_of_three_(C%2B%2B_programming)).

**Note**: Most of the time, you will not have to worry about any of this. If you use smart pointers and the STL (both of which use RAII and move semantics) you will hardly ever have to worry about implementing your own move semantics.

**Note**: There may be times when you want to force move semantics. You can do so with `std::move(x)`. By doing so, you are essentially telling the compiler "look, I know that this is an lvalue, but I want you to treat it as an rvalue for move semantics and I promise I won't use it anymore" (which would be bad because x will have NULL data in it after the move).

**Note**: Double ampersands (as in `auto&&`) do not always mean an rvalue reference! See [Scott Meyers' explanation](https://isocpp.org/blog/2012/11/universal-references-in-c11-scott-meyers) of the *universal reference* (as coined by Scott), officially known as a *forwarding reference*.

---

### Resource ###

- [SO: What are the differences between a pointer variable and a reference variable in C++](https://stackoverflow.com/questions/57483/what-are-the-differences-between-a-pointer-variable-and-a-reference-variable-in)
- [Wikipedia: Resource Acquisition Is Initialization (RAII)](https://en.wikipedia.org/wiki/Resource_acquisition_is_initialization)
- [Move semantics and rvalue references in C++11](https://www.cprogramming.com/c++11/rvalue-references-and-move-semantics-in-c++11.html)
- [SO: What are move semantics](https://stackoverflow.com/questions/3106110/what-are-move-semantics)
- [Lesson #5: Move Semantics](https://mbevin.wordpress.com/2012/11/20/move-semantics/)
- [C++11 rvalue References Explained](http://thbecker.net/articles/rvalue_references/section_01.html)
- [Simple code demonstrating copy vs move penalty](https://eli.thegreenplace.net/2011/12/15/understanding-lvalues-and-rvalues-in-c-and-c#rvalue-references-c-11)
