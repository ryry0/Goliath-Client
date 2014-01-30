SOURCE_EXT=cpp
HEAD_EXT=h
CC=g++
CFLAGS=-c  
LDFLAGS=

EXECUTABLE=Goliath_Client

SOURCES=$(wildcard *.$(SOURCE_EXT))
OBJECTS=$(SOURCES:.cpp=.o)

.PHONY: clean cleanall run test debug

all: $(SOURCES) $(EXECUTABLE)

debug: CFLAGS += -DDEBUG -g -Wall
debug: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(SOURCES) $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

%.o: %.cpp 
	$(CC) $(CFLAGS) $< -o $@

cleanall: clean
	rm -f $(EXECUTABLE)
	
proper: clean
	rm -f $(EXECUTABLE)

re: proper all

redo: proper debug
	
clean:
	rm -f *.o *.gch

run: 
	./$(EXECUTABLE)

test:
	gdb -tui $(EXECUTABLE)
