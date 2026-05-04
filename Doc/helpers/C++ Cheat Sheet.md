
## Table of Contents

- [[#Basics]]
- [[#Data Types]]
- [[#Control Flow]]
- [[#Functions]]
- [[#Pointers]]
- [[#References]]
- [[#Rvalue References & Move Semantics]]
- [[#Smart Pointers]]
- [[#Classes & Objects]]
- [[#Memory Management]]
- [[#Keywords Reference]]
- [[#Modern C++ (C++11/14/17/20)]]
- [[#STL Containers]]
- [[#Useful Snippets]]

---

## Basics

```cpp
#include <iostream>
using namespace std;

int main() {
    cout << "Hello, World!" << endl;
    return 0;
}
```

|Concept|Syntax|
|---|---|
|Comment|`// single line` / `/* block */`|
|Namespace|`using namespace std;`|
|Output|`cout << value << endl;`|
|Input|`cin >> variable;`|
|Compile|`g++ -std=c++20 file.cpp -o out`|

---

## Data Types

```cpp
int    a = 42;           // 32-bit integer
long   b = 1000000L;     // 64-bit integer
float  c = 3.14f;        // 32-bit float
double d = 3.14159;      // 64-bit float
char   e = 'A';          // single character
bool   f = true;         // boolean
auto   g = 42;           // type deduced by compiler (C++11)

// C++11 fixed-width types (preferred)
#include <cstdint>
int32_t  x = 100;
uint64_t y = 999ULL;
```

**Type casting:**

```cpp
int   a = (int)3.7;        // C-style cast
int   b = static_cast<int>(3.7);   // preferred C++ cast
float c = static_cast<float>(a);
```

---

## Control Flow

```cpp
// If / else
if (x > 0)       { /* ... */ }
else if (x == 0) { /* ... */ }
else             { /* ... */ }

// Switch
switch (x) {
    case 1:  cout << "one"; break;
    default: cout << "other";
}

// Ternary
int abs_val = (x >= 0) ? x : -x;

// Loops
for (int i = 0; i < 10; i++) { }
while (condition)             { }
do { } while (condition);

// Range-based for (C++11)
for (auto& elem : container) { }
```

---

## Functions

```cpp
// Declaration & definition
int add(int a, int b) { return a + b; }

// Default parameters
void greet(string name = "World") { cout << "Hello " << name; }

// Function overloading
int    square(int x)    { return x * x; }
double square(double x) { return x * x; }

// Inline function
inline int cube(int x) { return x * x * x; }

// Lambda (C++11)
auto multiply = [](int a, int b) -> int { return a * b; };

// Lambda capturing variables
int factor = 3;
auto scale = [factor](int x) { return x * factor; };   // capture by value
auto scale2 = [&factor](int x) { return x * factor; }; // capture by reference
```

---

## Pointers

> [!info] Core Concept A pointer stores the **memory address** of another variable. Use `*` to declare and dereference, `&` to get an address.

```cpp
int  value = 42;
int* ptr   = &value;   // ptr holds the address of value

cout << ptr;    // prints the address (e.g. 0x7ffee...)
cout << *ptr;   // dereference: prints 42
*ptr = 100;     // modifies value through pointer
```

### Pointer Arithmetic

```cpp
int arr[] = {10, 20, 30};
int* p = arr;        // points to arr[0]

p++;                 // now points to arr[1]
cout << *p;          // 20
cout << *(p + 1);    // 30
```

### Null Pointer (C++11)

```cpp
int* p = nullptr;    // prefer nullptr over NULL or 0
if (p != nullptr) { /* safe to use */ }
```

### Pointer to Pointer

```cpp
int  x   = 5;
int* p   = &x;
int** pp = &p;

cout << **pp;   // 5
```

### Const Pointers

```cpp
const int* p1 = &x;    // pointer to const: can't modify *p1
int* const p2 = &x;    // const pointer:    can't change address
const int* const p3 = &x; // both const
```

### Function Pointers

```cpp
int add(int a, int b) { return a + b; }

int (*funcPtr)(int, int) = &add;
cout << funcPtr(3, 4);   // 7

// Modern alias
using BinaryOp = int(*)(int, int);
BinaryOp op = add;
```

> [!info] See [[#Smart Pointers]] for `unique_ptr`, `shared_ptr`, and `weak_ptr` with full explanations.

---

## References

> [!info] Core Concept A reference is an **alias** for an existing variable. It must be initialized and cannot be reseated (unlike pointers).

```cpp
int  x = 10;
int& ref = x;    // ref is an alias for x

ref = 20;        // x is now 20
cout << x;       // 20
```

### Reference vs Pointer

|Feature|Pointer|Reference|
|---|---|---|
|Can be null|✅ `nullptr`|❌ must bind|
|Can be reseated|✅|❌|
|Needs dereferencing|✅ `*ptr`|❌ used directly|
|Syntax|`int* p = &x`|`int& r = x`|

### Pass by Reference vs Value

```cpp
void byValue(int x)  { x = 99; }   // original unchanged
void byRef(int& x)   { x = 99; }   // original modified
void byPtr(int* x)   { *x = 99; }  // original modified

// const reference — read-only, avoids copying large objects
void print(const string& s) { cout << s; }
```

---

## Rvalue References & Move Semantics

> [!info] Lvalue vs Rvalue
> 
> - An **lvalue** has a name and a persistent address — you can take its address with `&`. Examples: variables, array elements, dereferenced pointers.
> - An **rvalue** is a temporary with no persistent address — it lives only for the duration of an expression. Examples: literals (`42`), arithmetic results (`a + b`), return values from functions.

```cpp
int x = 42;      // x  is an lvalue (has a name, a place in memory)
int y = x + 1;   // (x + 1) is an rvalue — a temporary result
```

### Rvalue References `&&`

An rvalue reference binds **only** to temporaries. Its purpose is to allow you to "steal" resources from objects that are about to be destroyed, rather than copying them.

```cpp
int&&  rref  = 42;       // OK — binds to temporary
// int&& rref2 = x;      // ERROR — x is an lvalue

string&& s = string("hello");   // binds to temporary string
```

### Move Constructor & Move Assignment

Instead of copying heap-allocated data, a move operation **transfers ownership** — it takes the pointer and sets the source to null. This is O(1) vs O(n) for a deep copy.

```cpp
class Buffer {
    int*   data;
    size_t size;
public:
    // Regular constructor
    Buffer(size_t n) : size(n), data(new int[n]) {}

    // Destructor
    ~Buffer() { delete[] data; }

    // Copy constructor — deep copy, O(n)
    Buffer(const Buffer& other) : size(other.size), data(new int[other.size]) {
        copy(other.data, other.data + size, data);
    }

    // Move constructor — steal pointer, O(1)
    Buffer(Buffer&& other) noexcept
        : size(other.size), data(other.data) {
        other.data = nullptr;   // leave source in valid but empty state
        other.size = 0;
    }

    // Move assignment operator
    Buffer& operator=(Buffer&& other) noexcept {
        if (this != &other) {
            delete[] data;          // free our current resource
            data = other.data;      // steal
            size = other.size;
            other.data = nullptr;
            other.size = 0;
        }
        return *this;
    }
};
```

### `std::move`

`std::move` doesn't actually move anything — it **casts** an lvalue to an rvalue reference, signalling "I no longer need this value, you may steal it."

```cpp
Buffer a(100);
Buffer b = std::move(a);   // calls move constructor; a is now empty
// Using a after this is valid but dangerous — a.data == nullptr
```

### `std::forward` — Perfect Forwarding

Used in templates to forward arguments while preserving their value category (lvalue stays lvalue, rvalue stays rvalue).

```cpp
template<typename T>
void wrapper(T&& arg) {
    process(std::forward<T>(arg));   // forwards as lvalue or rvalue correctly
}
```

### The Rule of Five (C++11)

If you define **any** of these, you should define **all five**:

|Special Member|Purpose|
|---|---|
|Destructor|Free owned resources|
|Copy constructor|Deep copy|
|Copy assignment|Deep copy, handle self-assignment|
|Move constructor|Steal resources|
|Move assignment|Steal resources, handle self-assign|

```cpp
class MyClass {
public:
    ~MyClass();                              // destructor
    MyClass(const MyClass&);                 // copy constructor
    MyClass& operator=(const MyClass&);      // copy assignment
    MyClass(MyClass&&) noexcept;             // move constructor
    MyClass& operator=(MyClass&&) noexcept;  // move assignment
};
```

> [!tip] Rule of Zero The easiest rule of five is to **not need it**: use `std::string`, `std::vector`, and smart pointers as members. They handle their own memory, so your class needs no special members at all.

---

## Smart Pointers

> [!info] Why Smart Pointers? Raw `new`/`delete` is error-prone — you can forget to delete, throw before deleting, or delete twice. Smart pointers use **RAII**: the resource is automatically released when the smart pointer goes out of scope.

```cpp
#include <memory>
```

---

### `unique_ptr` — Exclusive Ownership

One and only one `unique_ptr` owns the resource at any time. When it goes out of scope (or is destroyed), the object is **automatically deleted**.

```cpp
// Creation — always use make_unique
auto ptr = make_unique<int>(42);
auto obj = make_unique<MyClass>("arg1", 2);   // forwards constructor args

// Dereference like a raw pointer
cout << *ptr;             // 42
obj->someMethod();

// Transfer ownership — original becomes null
auto ptr2 = std::move(ptr);
// ptr is now nullptr; ptr2 owns the object

// Release ownership — you get the raw pointer back and must manage it
int* raw = ptr2.release();   // ptr2 is now null
delete raw;                  // your responsibility now

// Replace the managed object
ptr2.reset(new int(99));     // old object deleted, now owns new one
ptr2.reset();                // or just delete and become null

// Check if non-null
if (ptr2) { cout << *ptr2; }
```

> [!warning] `unique_ptr` cannot be copied — only moved. This enforces sole ownership at compile time.

```cpp
auto a = make_unique<int>(1);
auto b = a;              // ❌ compile error — can't copy
auto c = std::move(a);   // ✅ ownership transferred
```

**Useful for:** local resources, factory functions, PIMPL idiom, class members that own heap data.

---

### `shared_ptr` — Shared Ownership

Multiple `shared_ptr`s can point to the same object. An internal **reference counter** tracks how many owners exist. The object is deleted when the **last** `shared_ptr` to it is destroyed.

```cpp
auto sp1 = make_shared<int>(10);
cout << sp1.use_count();   // 1

{
    auto sp2 = sp1;            // copy — both share ownership
    auto sp3 = sp1;
    cout << sp1.use_count();   // 3
}   // sp2 and sp3 go out of scope → ref count drops to 1

cout << sp1.use_count();   // 1
// when sp1 goes out of scope → ref count = 0 → object deleted

// Access
cout << *sp1;
sp1->method();

// Reset (drop this owner's share)
sp1.reset();               // ref count decremented; object deleted if 0
```

> [!warning] Circular References If two objects hold `shared_ptr`s to each other, neither ever reaches ref count 0 → **memory leak**. Break the cycle with `weak_ptr`.

```cpp
struct Node {
    shared_ptr<Node> next;   // ✅ fine for a chain
    shared_ptr<Node> parent; // ❌ creates a cycle if child and parent point at each other
    weak_ptr<Node>   parent; // ✅ use weak_ptr to break the cycle
};
```

**Useful for:** shared ownership in graphs/trees, caches, passing objects across threads.

---

### `weak_ptr` — Non-Owning Observer

A `weak_ptr` **observes** an object owned by `shared_ptr` without contributing to the reference count. It does **not** keep the object alive.

```cpp
auto sp = make_shared<int>(42);
weak_ptr<int> wp = sp;

cout << wp.use_count();    // 1 — wp doesn't count
cout << wp.expired();      // false — object still alive

// You CANNOT dereference a weak_ptr directly.
// You must "lock" it to get a temporary shared_ptr:
if (auto locked = wp.lock()) {   // returns shared_ptr; null if expired
    cout << *locked;             // safe to use
} else {
    cout << "Object no longer exists";
}

sp.reset();                // object deleted
cout << wp.expired();      // true
```

**Useful for:** back-pointers in trees, observer/listener patterns, caches that don't prevent eviction.

---

### Smart Pointer Comparison

|Feature|`unique_ptr`|`shared_ptr`|`weak_ptr`|
|---|---|---|---|
|Ownership|Sole|Shared|None|
|Reference counting|❌|✅|❌ (observes count)|
|Copyable|❌ (move only)|✅|✅|
|Overhead|Zero (vs raw ptr)|Small (control block)|Small|
|Dereferenceable|✅|✅|❌ (must lock first)|
|Use for|Sole ownership|Shared ownership|Break cycles / observe|

> [!tip] Default Choice Start with `unique_ptr`. Upgrade to `shared_ptr` only when you genuinely need shared ownership. Use `weak_ptr` to break `shared_ptr` cycles.

---

## Classes & Objects

### Class Definition

```cpp
class Animal {
private:
    string name;
    int    age;

public:
    // Constructor
    Animal(string n, int a) : name(n), age(a) {}

    // Destructor
    ~Animal() { cout << name << " destroyed\n"; }

    // Getter / Setter
    string getName() const { return name; }
    void   setAge(int a)   { age = a; }

    // Method
    void speak() const { cout << name << " makes a sound\n"; }
};

// Usage
Animal cat("Whiskers", 3);
cat.speak();
cout << cat.getName();
```

### Constructors

```cpp
class Point {
public:
    int x, y;

    Point()              : x(0), y(0) {}       // default
    Point(int x, int y)  : x(x), y(y) {}       // parameterized
    Point(const Point& p): x(p.x), y(p.y) {}   // copy
    Point(Point&& p) noexcept : x(p.x), y(p.y) {} // move (C++11)

    // Delegating constructor (C++11)
    Point(int v) : Point(v, v) {}
};
```

### Inheritance

```cpp
class Dog : public Animal {
private:
    string breed;

public:
    Dog(string n, int a, string b) : Animal(n, a), breed(b) {}

    // Override
    void speak() const override { cout << "Woof!\n"; }
};

// Polymorphism
Animal* pet = new Dog("Rex", 2, "Lab");
pet->speak();   // calls Dog::speak() if speak() is virtual
delete pet;
```

### Virtual Functions & Interfaces

```cpp
class Shape {
public:
    virtual double area() const = 0;   // pure virtual → abstract class
    virtual ~Shape() {}                // virtual destructor (always!)
};

class Circle : public Shape {
    double r;
public:
    Circle(double r) : r(r) {}
    double area() const override { return 3.14159 * r * r; }
};
```

### Operator Overloading

```cpp
class Vec2 {
public:
    float x, y;
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2& operator+=(const Vec2& o) { x+=o.x; y+=o.y; return *this; }
    bool operator==(const Vec2& o) const { return x==o.x && y==o.y; }
    friend ostream& operator<<(ostream& os, const Vec2& v) {
        return os << "(" << v.x << ", " << v.y << ")";
    }
};
```

### Static Members

```cpp
class Counter {
public:
    static int count;
    Counter()  { count++; }
    ~Counter() { count--; }
    static int getCount() { return count; }
};
int Counter::count = 0;   // definition outside class
```

---

## Keywords Reference

### `const` — Immutability

`const` prevents modification. Its **position** matters a lot.

```cpp
// --- On variables ---
const int x = 10;          // x cannot be changed

// --- On pointers (read right-to-left) ---
const int* p  = &x;        // pointer to const int    — can't modify *p, can change p
int* const p2 = &x;        // const pointer to int    — can modify *p2, can't change p2
const int* const p3 = &x;  // const pointer to const  — can't modify either

// --- On function parameters ---
void print(const string& s);       // read-only reference — no copy, no mutation
void process(const int* arr, int n); // can read arr[i], cannot write it

// --- On member functions (after the parentheses) ---
class Foo {
    int val = 0;
public:
    int  get() const { return val; }  // promises not to modify *this
    void set(int v)  { val = v; }     // non-const — modifies *this
};

const Foo f;
f.get();   // ✅ const method on const object
f.set(1);  // ❌ compile error

// --- constexpr (C++11) — evaluated at compile time ---
constexpr int SIZE = 256;
constexpr int square(int x) { return x * x; }
int arr[square(4)];   // valid — known at compile time
```

> [!tip] `const` placement rule **East const** style: `int const*` = pointer to const int. Read the declaration **right to left**: "pointer to const int". Many prefer this for consistency.

---

### `static` — Lifetime & Linkage

`static` has different meanings depending on context.

```cpp
// --- Static local variable — initialized once, lives for the whole program ---
void counter() {
    static int count = 0;   // not re-initialized on each call
    count++;
    cout << count;
}
counter(); // 1
counter(); // 2
counter(); // 3

// --- Static member variable — shared across ALL instances ---
class Dog {
public:
    static int total;    // declaration
    Dog()  { total++; }
    ~Dog() { total--; }
};
int Dog::total = 0;      // definition (required outside class)

// --- Static member function — no `this` pointer, called on the class ---
class MathUtils {
public:
    static int add(int a, int b) { return a + b; }
};
MathUtils::add(3, 4);   // no instance needed

// --- Static at file scope — internal linkage (not visible to other .cpp files) ---
static int helper = 42;         // in a .cpp file
static void internalFunc() {}   // only visible in this translation unit
// Modern equivalent: use anonymous namespaces
namespace {
    int helper2 = 99;
}
```

---

### `volatile` — Prevent Optimisation

Tells the compiler that a variable may change **outside the program's control** (hardware register, signal handler, another thread via low-level code). The compiler must read/write it every time — no caching in registers.

```cpp
volatile int sensorValue;          // hardware register
volatile bool stopFlag = false;    // set by signal handler

// Without volatile, the compiler might optimise this loop away:
while (!stopFlag) {
    // do work
}
// With volatile, it re-reads stopFlag every iteration.
```

> [!warning] `volatile` is NOT a substitute for proper thread synchronisation. Use `std::atomic` or mutexes for thread-safe access.

---

### `mutable` — Const Exception

Allows a member to be modified **even inside a `const` method**. Useful for caches, lazy evaluation, and mutex locks.

```cpp
class ExpensiveCalc {
    mutable int  cachedResult = -1;   // mutable — can change in const method
    mutable bool dirty        = true;
    int          data         = 42;
public:
    int getResult() const {
        if (dirty) {
            cachedResult = data * data;   // allowed because mutable
            dirty = false;
        }
        return cachedResult;
    }
};

// Common pattern: mutable mutex in const method
class ThreadSafeCache {
    mutable mutex mtx;
    mutable map<int,int> cache;
public:
    int get(int key) const {
        lock_guard<mutex> lock(mtx);   // mtx is mutable → allowed here
        return cache[key];
    }
};
```

---

### `explicit` — Prevent Implicit Conversion

Prevents the compiler from using a constructor for **implicit type conversion**.

```cpp
class Wrapper {
public:
    explicit Wrapper(int x) { }   // must be called explicitly
};

Wrapper w1(42);      // ✅ direct initialization
Wrapper w2 = 42;     // ❌ implicit conversion blocked
void foo(Wrapper);
foo(42);             // ❌ blocked — must write foo(Wrapper(42))

// Without explicit:
class Bad {
public:
    Bad(int x) {}    // allows silent implicit conversion — often unintended
};
Bad b = 42;          // ✅ compiles — might be a surprise bug
```

---

### `inline` — Suggest Inlining

Hints to the compiler to **expand the function body at the call site** instead of making an actual function call. Also allows a function to be defined in a header without ODR (One Definition Rule) violations.

```cpp
inline int square(int x) { return x * x; }   // may expand inline

// In headers: inline avoids "multiple definition" linker errors
// when included in multiple .cpp files
inline void logMessage(const string& s) { cerr << s << "\n"; }
```

> [!info] Modern compilers inline automatically based on heuristics. `inline` today is more relevant for **ODR purposes in headers** than for performance.

---

### `noexcept` — No-Exception Guarantee

Declares that a function **will not throw** an exception. The compiler can generate more efficient code, and it's required for move operations to be used by STL containers.

```cpp
int safeDiv(int a, int b) noexcept { return a / b; }   // promises no throw

// Critical for move semantics — STL uses move only if noexcept
Buffer(Buffer&& other) noexcept;
Buffer& operator=(Buffer&&) noexcept;

// noexcept(condition) — conditional
template<typename T>
void swap(T& a, T& b) noexcept(noexcept(T(std::move(a)))) { ... }

// Check at compile time
static_assert(noexcept(safeDiv(1, 2)));
```

> [!warning] If a `noexcept` function **does** throw, `std::terminate()` is called — the program aborts immediately. Only mark `noexcept` when you're certain.

---

### `override` & `final`

```cpp
class Base {
public:
    virtual void draw() const;
    virtual void resize(int n);
};

class Derived : public Base {
public:
    void draw() const override;    // ✅ override: compiler checks Base has this
    // void drwa() const override; // ❌ compile error: typo caught immediately
    void resize(int n) override final; // final: no further subclass can override this
};

class Base2 final { };   // final on a class: cannot be inherited from
// class Child : public Base2 {}; // ❌ compile error
```

---

### `delete` & `default` — Special Member Control

```cpp
class NonCopyable {
public:
    NonCopyable() = default;                      // compiler-generated default ctor
    NonCopyable(const NonCopyable&) = delete;     // forbid copying
    NonCopyable& operator=(const NonCopyable&) = delete;
};

class AlwaysInt {
public:
    AlwaysInt(int x) {}
    AlwaysInt(double) = delete;   // forbid double → int implicit conversion
};

AlwaysInt a(42);    // ✅
AlwaysInt b(3.14);  // ❌ deleted
```

---

### Keywords Quick-Reference Table

|Keyword|Where it goes|What it does|
|---|---|---|
|`const`|variable, param, method (after `()`)|Prevents modification|
|`constexpr`|variable or function|Evaluated at compile time|
|`static`|local var, member, file scope|Persistent lifetime / shared / internal linkage|
|`volatile`|variable|Always re-read from memory, no compiler caching|
|`mutable`|class member|Modifiable even in `const` methods|
|`explicit`|constructor / conversion operator|Blocks implicit conversions|
|`inline`|function / variable (C++17)|ODR-safe in headers; hint to inline|
|`noexcept`|function signature|Guarantees no throw; enables optimisations|
|`override`|virtual method in derived class|Compile-time check that base has this method|
|`final`|virtual method or class|Prevents further overriding / inheritance|
|`delete`|special member function|Explicitly disables that function|
|`default`|special member function|Requests compiler-generated implementation|
|`virtual`|member function|Enables runtime polymorphism|
|`friend`|function or class declaration|Grants access to private members|
|`extern`|variable or function|Declares symbol defined in another translation unit|
|`typename`|template parameter / nested type|Disambiguates types in templates|

```cpp
// Stack allocation — auto-freed
int x = 10;

// Heap allocation — must free manually
int*    p   = new int(42);
int*    arr = new int[10];
delete  p;
delete[] arr;

// RAII pattern — prefer smart pointers
auto p2  = make_unique<int>(42);   // auto-deleted when out of scope
auto arr2 = make_unique<int[]>(10);
```

> [!warning] Common Pitfalls
> 
> - **Memory leak**: `new` without `delete`
> - **Dangling pointer**: using pointer after `delete`
> - **Double free**: calling `delete` twice on same pointer
> - **Buffer overflow**: writing past array bounds ✅ Prefer `make_unique` / `make_shared` to avoid all of these.

---

## Modern C++ (C++11/14/17/20)

### C++11 Essentials

```cpp
auto x = 3.14;                     // type inference
auto [a, b] = make_pair(1, 2);     // structured bindings (C++17)

// Range-based for
for (const auto& item : vec) { }

// nullptr
int* p = nullptr;

// Lambda
auto sq = [](int x) { return x * x; };

// Move semantics
vector<int> a = {1,2,3};
vector<int> b = std::move(a);      // a is now empty

// Initializer lists
vector<int> v = {1, 2, 3, 4, 5};
```

### C++17 Highlights

```cpp
// if with initializer
if (auto it = map.find(key); it != map.end()) {
    cout << it->second;
}

// Structured bindings
auto [key, val] = *map.begin();

// std::optional
#include <optional>
optional<int> maybeVal = 42;
if (maybeVal) cout << *maybeVal;
maybeVal = nullopt;    // empty

// std::variant
#include <variant>
variant<int, string, float> v = "hello";
cout << get<string>(v);

// std::string_view (non-owning)
void print(string_view sv) { cout << sv; }
```

### C++20 Highlights

```cpp
// Concepts — constrain templates
#include <concepts>
template<typename T>
requires std::integral<T>
T double_it(T x) { return x * 2; }

// or shorthand
auto square(std::integral auto x) { return x * x; }

// Ranges
#include <ranges>
auto v = views::iota(1, 10)
       | views::filter([](int x){ return x % 2 == 0; })
       | views::transform([](int x){ return x * x; });

// Coroutines (basic)
#include <coroutine>
// (complex — see std::generator in C++23)

// std::format (like Python f-strings)
#include <format>
string s = format("Hello, {}! You are {} years old.", name, age);

// Three-way comparison (spaceship operator)
auto result = (a <=> b);   // returns strong_ordering / partial_ordering
```

### Templates

```cpp
// Function template
template<typename T>
T maxVal(T a, T b) { return (a > b) ? a : b; }

// Class template
template<typename T, int N>
class Array {
    T data[N];
public:
    T& operator[](int i) { return data[i]; }
    int size() const { return N; }
};

Array<int, 5> arr;

// Variadic template (C++11)
template<typename... Args>
void print(Args... args) { (cout << ... << args); }  // fold expression (C++17)
```

---

## STL Containers

```cpp
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <stack>
#include <queue>

// vector — dynamic array
vector<int> v = {3,1,2};
v.push_back(4);
sort(v.begin(), v.end());

// map — sorted key-value
map<string,int> m;
m["alice"] = 30;
m.contains("alice");       // C++20

// unordered_map — hash map, O(1) avg
unordered_map<string,int> um;

// set / unordered_set
set<int> s = {3,1,2};      // sorted, unique

// stack / queue
stack<int> stk;  stk.push(1); stk.top(); stk.pop();
queue<int> q;    q.push(1);   q.front(); q.pop();

// pair / tuple
auto p = make_pair(1, "hello");
auto t = make_tuple(1, 2.5, "world");
auto [x, y, z] = t;           // C++17 structured binding
```

### Common Algorithms

```cpp
#include <algorithm>
#include <numeric>

sort(v.begin(), v.end());
sort(v.begin(), v.end(), greater<int>());   // descending

auto it = find(v.begin(), v.end(), 3);
bool found = binary_search(v.begin(), v.end(), 3);

int sum = accumulate(v.begin(), v.end(), 0);

auto max_it = max_element(v.begin(), v.end());
auto min_it = min_element(v.begin(), v.end());

reverse(v.begin(), v.end());
unique(v.begin(), v.end());    // remove consecutive duplicates

// C++20 ranges
ranges::sort(v);
auto even = v | views::filter([](int x){ return x%2==0; });
```

---

## Useful Snippets

### String Operations

```cpp
#include <string>
#include <sstream>

string s = "Hello World";
s.length();           // 11
s.substr(0, 5);       // "Hello"
s.find("World");      // 6
s.replace(6, 5, "C++"); // "Hello C++"
s += "!";             // append

// String to int / int to string
int    n = stoi("42");
string t = to_string(42);

// Split by delimiter
stringstream ss("a,b,c");
string token;
while (getline(ss, token, ',')) { cout << token << "\n"; }
```

### File I/O

```cpp
#include <fstream>

// Write
ofstream out("file.txt");
out << "Hello\n";
out.close();

// Read
ifstream in("file.txt");
string line;
while (getline(in, line)) { cout << line << "\n"; }
```

### Exception Handling

```cpp
try {
    throw runtime_error("Something went wrong");
} catch (const runtime_error& e) {
    cerr << "Error: " << e.what() << "\n";
} catch (...) {
    cerr << "Unknown error\n";
}
```

### Timing Code

```cpp
#include <chrono>
using namespace chrono;

auto start = high_resolution_clock::now();
// ... code ...
auto end   = high_resolution_clock::now();
auto ms    = duration_cast<milliseconds>(end - start).count();
cout << "Elapsed: " << ms << " ms\n";
```

---

> [!tip] Quick Reference
> 
> - Prefer **smart pointers** over raw pointers
> - Prefer **references** for function params (use `const&` for read-only)
> - Use **`auto`** to reduce verbosity, but don't sacrifice clarity
> - Use **`override`** and **`final`** on virtual methods
> - Use **`nullptr`** instead of `NULL`
> - Use **`=delete`** / **`=default`** to control special member functions