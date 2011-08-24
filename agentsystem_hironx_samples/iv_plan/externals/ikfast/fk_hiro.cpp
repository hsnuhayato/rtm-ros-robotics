#include <iostream>
#include <stdlib.h>

typedef double IKReal;

using namespace std;

#define IKFAST_API
IKFAST_API void fk(const IKReal* j, IKReal* eetrans, IKReal* eerot);

#ifndef IKFAST_NO_MAIN

int main (int argc, char** argv)
{
  IKReal js[6];
  IKReal eetrans[3];
  IKReal eerot[3*3];

  for (int i = 0; i < 6; i++) {
    js[i] = atof(argv[i+1]);
  }

  fk(js, eetrans, eerot);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cout << eerot[i*3+j] << " ";
    }
    cout << eetrans[i] << " ";
  }

  return 0;
}

#endif
