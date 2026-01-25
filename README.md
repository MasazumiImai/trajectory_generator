# trajectory_generator

## How to Build

### C++

```bash
mkdir -p build && cd build
cmake .. && make && sudo make install
```

## How to Test

### C++

```bash
mkdir -p build && cd build
cmake .. -DBUILD_TESTING=ON && make
ctest -V
```
