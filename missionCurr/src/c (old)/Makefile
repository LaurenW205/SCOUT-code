CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall

# pkg-config package name 
PKG := opencv4

OPENCV_CFLAGS := $(shell pkg-config --cflags $(PKG))
OPENCV_LIBS   := $(shell pkg-config --libs $(PKG))

SRCS := $(wildcard *.cpp)
OBJS := $(SRCS:.cpp=.o)
BIN  := main

.PHONY: all clean

all: $(BIN)

$(BIN): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@ $(OPENCV_LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -c $< -o $@

clean:
	$(RM) $(OBJS) $(BIN)
