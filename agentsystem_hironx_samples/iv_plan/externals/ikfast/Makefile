all: libik_hiro.so 

ik_pa10: ik_pa10.cpp
	g++ -o $@ $< -O2

ik_hiro1: hiroreal_rightarm_ikfast1.cpp
	g++ -lstdc++ -llapack -o $@ $< -O2

ik_hiro2: hiroreal_rightarm_ikfast2.cpp
	g++ -lstdc++ -llapack -o $@ $< -O2

ik_hiro3: hirobest_rightarm_ikfast.cpp
	g++ -lstdc++ -llapack -o $@ $< -O2

ik_hiro: hironx_ikfast42.Transform6D.3_4_5_6_7_8.cpp
	g++ -lstdc++ -llapack -o $@ $< -O2

libik_hiro.so: hironx_ikfast42.Transform6D.3_4_5_6_7_8.cpp
	(g++ -fPIC -llapack -lstdc++ -lboost_python -I/usr/include/python2.6 -DIKFAST_NO_MAIN -shared \
	-Wl,-soname,$@ -o $@ $< && \
	cp $@ ../../lib)

#fk_hiro: fk_hiro.o
#	g++ -o $@ $^ -lik_hiro -L.

fk_hiro: fk_hiro.cpp hironx_ikfast42.Transform6D.3_4_5_6_7_8.cpp
	g++ -llapack -lstdc++ -DIKFAST_NO_MAIN -o $@ $^

# .cpp.o:
# 	g++ -c $<

clean:
	rm -f libik_hiro.so
