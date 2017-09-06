CXX = g++
CPPFLAGS =        # put pre-processor settings (-I, -D, etc) here
CXXFLAGS = -Wall  # put compiler settings here
LDFLAGS =         # put linker settings here

readmcregs: readmcregs.o
	$(CXX) -o $@ $(CXXFLAGS) $(LDFLAGS) readmcregs.o

.cpp.o:
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $<

