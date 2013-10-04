#include <iostream>
#include "Hello.hh"

class World_impl: public POA_Hello::World,
                  public PortableServer::RefCountServantBase
{
  virtual char * hello ()
  {
    return CORBA::string_dup ("Greetings from C++ World server!!");
  }
};

int main (int argc, char **argv)
{
  try {
    // Initialize the ORB
    CORBA::ORB_var orb = CORBA::ORB_init (argc, argv);

    // Get the Root POA
    CORBA::Object_var obj = orb->resolve_initial_references ("RootPOA");
    PortableServer::POA_var poa = PortableServer::POA::_narrow (obj);

    // Activate POA manager
    PortableServer::POAManager_var mgr = poa->the_POAManager ();
    mgr->activate ();

    // Create the servant(s)
    //World_impl world_servant;
    World_impl world_servant;

    // Activate object implicitly by calling _this()
    Hello::World_var world = world_servant._this ();

    // Write stringified reference to stdout
    CORBA::String_var str = orb->object_to_string (world);
    std::cout << str << std::endl;

    // Accept requests
    orb->run ();
  }
  catch (const CORBA::Exception & e)
  {
    std::cerr << "Server caught CORBA exception: " << std::endl;
    return 100;
  }
  return 0;
}

