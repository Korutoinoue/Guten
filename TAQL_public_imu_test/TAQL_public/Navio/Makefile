CXX = g++              #compiling  command, use to Cxx can be defined as gcc or g++ etc  for the compiling convience 
CFLAGS = -std=c++11 -Wno-psabi -c -I .

SRC = $(wildcard */*.cpp)
OBJECTS = $(SRC:.cpp=.o)                     

%.o: %.cpp                                    #xx.o   what contained in the folder and can be made into by a xx.c files                            #test: dependencies
	$(CXX) $(CFLAGS) -o $@ $<                #  $() is the part will be replaced in Cxx                                                     #gcc test.c -o test  

all: $(OBJECTS)                                                                   
	ar rcs libnavio.a $(OBJECTS)

clean:
	rm -f */*.o *.a








	#multiple makefile