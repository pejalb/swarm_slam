CPP = g++
INCLUDES = -I/usr/local/Aria/include -I../eigen/
LIBS = -L/usr/local/Aria/lib 
FLAGS = -fPIC -lAria -lpthread -ldl -lrt -std=c++11 -lstdc++ -fpermissive -DCMAKE_CXX_FLAGS=-D_GLIBCXX_USE_CXX11_ABI=0
SOURCE = gridMap.cpp ponto.cpp pose.cpp pso.cpp scanMatch.cpp slam.cpp
HEADERS = ponto.h  pose.h  pso.h  scanMatch.h  slam.h  constantes.h
swarm-slam : $(HEADERS) $(SOURCE)  teste_auto_ciclo.cpp
	$(CPP) $(FLAGS) $(INCLUDES) $(LIBS) $^ -o $@
.PHONY : clean
clean:
	rm -rf *.gch
	rm -rf swarm-slam