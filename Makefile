CC=g++
CONFIG=$(shell llvm-config --cxxflags --ldflags --libs all --system-libs)

default: FusionRegAlloc_one.cpp
	$(CC) -c -g FunctionPassSequence.cpp $(CONFIG) -o FunctionPassSequence.o
	$(CC) -c -g PhaseAnalyserPass.cpp $(CONFIG) -o PhaseAnalyserPass.o
	ar rvs libPhase.a FunctionPassSequence.o PhaseAnalyserPass.o
	$(CC) -g Main.cpp libPhase.a $(CONFIG) -L. -lPinhaoStaticProfiler -o Main.out
