CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra

.PHONY: all stack shape queue clean

all: stack shape queue

stack: stack/bounded_stack.cpp
	$(CXX) $(CXXFLAGS) -o stack.out stack/bounded_stack.cpp

shape: shape/shape_main.cpp shape/circle.cpp shape/rectangle.cpp
	$(CXX) $(CXXFLAGS) -I shape -o shape.out shape/shape_main.cpp shape/circle.cpp shape/rectangle.cpp

queue: queue.cpp
	$(CXX) $(CXXFLAGS) -o queue.out queue.cpp

clean:
	rm -f stack.out shape.out queue.out
