
CFLAGS += -I/usr/include/python2.6
CFLAGS += -Iexternals/pqp-1.3/PQP_v1.3/include/

all: externals idlcompile

externals:
	$(MAKE) -C externals

idlcompile: ArmPlan.idl GraspController.idl
	omniidl -bpython $^

cpqp.so: cpqp.o
	g++ -shared -g -o $@ $^ -lboost_python -lPQP -Lexternals/pqp-1.3/PQP_v1.3/lib

cpqp.o: cpqp.cpp
	g++ -g -c $^ -fPIC $(CFLAGS)

clean:
	rm -rf _GlobalIDL _GlobalIDL__POA *.pyc parsetab.py
