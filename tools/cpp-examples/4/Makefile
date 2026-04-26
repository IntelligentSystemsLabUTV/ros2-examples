CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -pthread

.PHONY: all queue_v4 clean

all: queue_v4

queue_v4: queue_v4.cpp queue_v4.hpp
	$(CXX) $(CXXFLAGS) -o queue_v4.out queue_v4.cpp

clean:
	rm -f queue_v4.out
