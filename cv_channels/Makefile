all: main

main: main.cpp
	g++ -std=c++11 -Wall $^ `pkg-config --libs --cflags opencv` -o $@

clean:
	rm main
