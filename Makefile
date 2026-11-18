# Everything runs inside the project venv. Heavy steps use nice so the machine stays usable.
PY      := .venv/bin/python
NICE    := nice -n 10
CXX     ?= c++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra

.PHONY: data build test run report clean

$(PY):
	uv venv .venv --python 3.12 || python3 -m venv .venv
	uv pip install --python $(PY) -r requirements.txt || $(PY) -m pip install -r requirements.txt

data: data/monza_lap.csv
data/monza_lap.csv: | $(PY)
	$(NICE) $(PY) src/fetch_data.py

build: build/liblapsim.so build/test_lapsim
build/liblapsim.so: cpp/lapsim.cpp cpp/lapsim.hpp
	@mkdir -p build
	$(NICE) $(CXX) $(CXXFLAGS) -shared -fPIC cpp/lapsim.cpp -o $@
build/test_lapsim: cpp/test_lapsim.cpp cpp/lapsim.cpp cpp/lapsim.hpp
	@mkdir -p build
	$(NICE) $(CXX) $(CXXFLAGS) cpp/test_lapsim.cpp cpp/lapsim.cpp -o $@

test: build data
	./build/test_lapsim
	$(NICE) $(PY) -m pytest -q tests

run: build data
	$(NICE) $(PY) src/run.py

report: run
	$(NICE) $(PY) src/report.py

clean:
	rm -rf build results/run.npz .pytest_cache src/__pycache__ tests/__pycache__
