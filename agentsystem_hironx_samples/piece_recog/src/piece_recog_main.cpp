// -*- C++ -*-
/*!
 * Ryo Hanai <hanai@jsk.t.u-tokyo.ac.jp>
 */

#include "piece_recog.h"

int main (int argc, char** argv) {
    if (argc != 3)
	return -1;

    std::string imgfile = argv[1];
    std::string piece = argv[2];

    recognize (imgfile, piece, false);
    return 0;
}
