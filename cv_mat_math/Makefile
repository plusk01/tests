all: test_stdlib_math test_cv_math

test_stdlib_math: stdlib_container_math.cpp
	g++ -std=c++11 -Wall $^ -o $@ -lopencv_core

test_cv_math: cv_math.cpp
	g++ -std=c++11 -Wall $^ `pkg-config --libs --cflags opencv` -o $@

clean:
	rm test_stdlib_math test_cv_math
