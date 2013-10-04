all: server client Hello_idl.py
	@echo ";; make sure to restart name server, sudo /etc/init.d/omniorb4-nameserver restart"
	@echo ";; ./server"
	@echo ";; ./client IOB:01....."
	@echo ";; client.py IOB:01....."

HelloSK.cc: Hello.idl
	omniidl -bcxx Hello.idl

Hello: Hello.idl
	idlj -fall Hello.idl

server: server.cpp HelloSK.cc
	gcc -o server server.cpp HelloSK.cc `pkg-config omniORB4 --cflags --libs`

client: client.cpp HelloSK.cc
	gcc -o client client.cpp HelloSK.cc `pkg-config omniORB4 --cflags --libs`

Hello_idl.py:
	omniidl -bpython Hello.idl


clean:
	rm -fr Hello Hello__POA Hello_idl.py *.class *.hh *.cc *.pyc omninames-* client server *~

