# multi-objective-mod-v3.1
Ridesharing system for multiple receiving calls with system user id

### Environment setting
```bash
conda env create --file env.yaml
```

### Navigation module
It is fur using my navigation algorithm (``./astar/planner.cpp``) in Python3. Pybind11 is a header library facilitating 
seamless C++ and Python interoperability with minimal code, making it straightforward to expose C++ functions and 
classes to Python.
#### Compile
```bash
c++ -O3 -Wall -shared -std=c++11 -fPIC $(python3 -m pybind11 --includes) ./astar/astar.h ./astar/astar.cpp ./astar/planner.cpp -o planner.so
```

### Inputs


### Example 1
```python
python3 main.py -w 0 1 0 -p 10 -d 1.5
```
### Example 2
```python
python3 main.py -w 0 1 0 -p 10 -d 2.0
```