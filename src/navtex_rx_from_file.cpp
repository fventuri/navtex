/* -*- c++ -*- */
/*
 * Copyright 2020 Franco Venturi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// decode a NAVTEX sound file (signed LE16 sampled at 11025Hz)
// NOTE: a different sample rate (for instance 48kHz) works too
//       (see examples in the README file)

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include "navtex_rx.h"

constexpr int BUFSIZE = 8192;

int main(int argc, const char** argv)
{
    auto inbuf = new short[BUFSIZE];

    int sample_rate = 11025;
    if (argc >= 2) {
        if (sscanf(argv[1], "%d", &sample_rate) != 1) {
            fprintf(stderr, "invalid sample rate: %s\n", argv[1]);
            exit(EXIT_FAILURE);
        }
    }
    
    int fd;
    if (argc == 2 || strcmp(argv[2], "-") == 0) {
        fd = fileno(stdin);
    } else {
        int flags = O_RDONLY;
#ifdef O_BINARY
        flags |= O_BINARY;
#endif
        fd = open(argv[2], flags);
        if (fd == -1) {
            fprintf(stderr, "open(%s) failed: %s\n", argv[2], strerror(errno));
            exit(EXIT_FAILURE);
        }
    }

    // disable buffering on stdout
    setvbuf(stdout, nullptr, _IONBF, 0);

    bool only_sitor_b = false;
    bool reverse = false;
    navtex_rx nv(sample_rate, only_sitor_b, reverse, stdout);

    while (true) {
        auto nread = read(fd, inbuf, BUFSIZE * sizeof(short));
        if (nread < 0) {
            fprintf(stderr, "read() failed: %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        }
        if (nread == 0)
            break;
        int nb_samples = nread / sizeof(short);
        nv.process_data(inbuf, nb_samples);
    }
    fflush(stdout);

    if (fd != fileno(stdin))
        close(fd);

    return 0;
}
