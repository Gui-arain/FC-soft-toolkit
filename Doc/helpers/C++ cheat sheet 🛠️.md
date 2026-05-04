
## Table of Contents

- [[#Basics]]
- [[#Data Types]]
- [[#Control Flow]]
- [[#Functions]]
- [[#Pointers]]
- [[#References]]
- [[#Classes & Objects]]
- [[#Memory Management]]
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

***Note:*** the type of the pointer (`int` in the above example) matches the type of the data it is pointing to (`42` in the example)
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

***Note:*** If new pointer not initialised with a correct address or `nullptr` with might end up writing in an❗️undesired memory space.
You can also write:

```cpp
int* p{}; //Implicitly initialise with nullptr
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

### Smart Pointers (C++11) ⭐

```cpp
#include <memory>

// unique_ptr — sole ownership, auto-deleted
auto up = make_unique<int>(42);
cout << *up;

// shared_ptr — shared ownership, ref-counted
auto sp1 = make_shared<int>(10);
auto sp2 = sp1;            // both share ownership
cout << sp1.use_count();   // 2

// weak_ptr — non-owning reference (breaks cycles)
weak_ptr<int> wp = sp1;
if (auto locked = wp.lock()) { cout << *locked; }
```
Need extra explanation

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
***Note:*** Using const references as functions parameters allows to directly access the data in memory without any copying
### Rvalue References (C++11) — Move Semantics

```cpp
int&& rref = 42;   // rvalue reference

// Move constructor example
class Buffer {
    int* data;
public:
    Buffer(Buffer&& other) noexcept : data(other.data) {
        other.data = nullptr;   // steal the resource
    }
};

// std::move casts to rvalue
Buffer a;
Buffer b = std::move(a);   // moves, doesn't copy
```
 
 need explanation

---

## Memory Management

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

***Note:*** It is recommended to set the deleted pointer to `nullptr` to prevent dangling pointers:
```cpp
delete p;
p = nullptr;
```

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