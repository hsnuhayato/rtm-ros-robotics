CFLAGS = -g -O2 -Wall

all: cpqp.so

cpqp.so: cpqp.o
	g++ -shared -g -o $@ $^ -lboost_python -lPQP -L../externals/PQP_v1.3/lib

cpqp.o: cpqp.cpp
	g++ $(CFLAGS) -c $< -fPIC -I/usr/include/python2.6 -I../externals/PQP_v1.3/include

clean:
	rm -rf cpqp.so cpqp.o *.pyc
