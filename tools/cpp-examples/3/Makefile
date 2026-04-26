CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -pthread

.PHONY: all threads shared_queue queue_v3 clean

all: threads shared_queue queue_v3

threads: threads.cpp
	$(CXX) $(CXXFLAGS) -o threads.out threads.cpp

shared_queue: shared_queue.cpp
	$(CXX) $(CXXFLAGS) -o shared_queue.out shared_queue.cpp

queue_v3: queue_v3.cpp queue_v3.hpp
	$(CXX) $(CXXFLAGS) -o queue_v3.out queue_v3.cpp

clean:
	rm -f threads.out shared_queue.out queue_v3.out
