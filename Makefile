CXXFLAGS=-Wall

apollonian: apollonian.cpp apollonian.h
	$(CXX) $(CXXFLAGS) -ggdb -o apollonian apollonian.cpp
.PHONY: clean
clean:
	@rm apollonian >/dev/null 2>&1 || echo nothing to do
