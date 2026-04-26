CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra

.PHONY: all templates smart_pointers callbacks queue_v2 clean

all: templates smart_pointers callbacks queue_v2

templates: templates.cpp
	$(CXX) $(CXXFLAGS) -o templates.out templates.cpp

smart_pointers: smart_pointers.cpp
	$(CXX) $(CXXFLAGS) -o smart_pointers.out smart_pointers.cpp

callbacks: callbacks.cpp
	$(CXX) $(CXXFLAGS) -o callbacks.out callbacks.cpp

queue_v2: queue_v2.cpp queue_v2.hpp
	$(CXX) $(CXXFLAGS) -o queue_v2.out queue_v2.cpp

clean:
	rm -f templates.out smart_pointers.out callbacks.out queue_v2.out
