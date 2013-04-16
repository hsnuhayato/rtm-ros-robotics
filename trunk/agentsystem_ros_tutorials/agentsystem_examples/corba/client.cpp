#include <iostream>
#include "Hello.hh"

int
main (int argc, char **argv)
{
  try {
    // Initialize the ORB
    CORBA::ORB_var orb = CORBA::ORB_init (argc, argv);

    // Expect IOR as command-line argument
    if (argc != 2) {
      std::cerr << "Usage: client IOR" << std::endl;
      throw 0;			// neat trick used to exit try block
    }

    // Destringify the IOR
    CORBA::Object_var obj = orb->string_to_object (argv[1]);
    if (CORBA::is_nil (obj)) {
      std::cerr << "Nil World reference" << std::endl;
      throw 0;
    }

    // Narrow an Object to a World
    Hello::World_var world = Hello::World::_narrow (obj);
    if (CORBA::is_nil (world)) {
      std::cerr << "Invalid World reference" << std::endl;
      throw 0;
    }

    // Say hello to the world
    CORBA::String_var s = world->hello ();
    std::cout << "World said \"" << s << "\"" << std::endl;
  }
  catch (const CORBA::Exception & e)
  {
    std::cerr << "Client caught CORBA exception: " << std::endl;
    return 100;
  }
  catch ( ...) {
    // Logical error; message already displayed
    return 1;
  }
  return 0;
}
