all: ik_pa10 ik_hiro1 ik_hiro2 ik_hiro3 libik_hiro.so fk_hiro

ik_pa10: ik_pa10.cpp
	g++ -o $@ $< -O2

ik_hiro1: hiroreal_rightarm_ikfast1.cpp
	g++ -lstdc++ -llapack -o $@ $< -O2

ik_hiro2: hiroreal_rightarm_ikfast2.cpp
	g++ -lstdc++ -llapack -o $@ $< -O2

ik_hiro3: hirobest_rightarm_ikfast.cpp
	g++ -lstdc++ -llapack -o $@ $< -O2

libik_hiro.so: hirobest_rightarm_ikfast.cpp
	g++ -fPIC -llapack -lstdc++ -DIKFAST_NO_MAIN -shared \
	-Wl,-soname,$@ -o $@ $<

#fk_hiro: fk_hiro.o
#	g++ -o $@ $^ -lik_hiro -L.

fk_hiro: fk_hiro.cpp hirobest_rightarm_ikfast.cpp
	g++ -llapack -lstdc++ -DIKFAST_NO_MAIN -o $@ $^

# .cpp.o:
# 	g++ -c $<

clean:
	rm -f ik_pa10 ik_hiro1 ik_hiro2 ik_hiro3 libik_hiro.so fk_hiro
