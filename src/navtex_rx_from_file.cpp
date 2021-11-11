/* -*- c++ -*- */
/*
 * Copyright 2020 Franco Venturi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// decode a NAVTEX sound file (signed LE16 sampled at 11025Hz)
// NOTE: a different sample rate (for instance 48kHz) works too, as long
// as the variable 'sample_rate' is set to the correct value (line 39 or 40)

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include "navtex_rx.h"

constexpr int BUFSIZE = 8192;

int main(int argc, const char** argv)
{
    auto inbuf = new short[BUFSIZE];

    int fd;
    if (argc == 1 || strcmp(argv[1], "-") == 0) {
        fd = fileno(stdin);
    } else {
        fd = open(argv[1], O_RDONLY);
        if (fd == -1) {
            fprintf(stderr, "open(%s) failed: %s\n", argv[1], strerror(errno));
            exit(EXIT_FAILURE);
        }
    }

    // disable buffering on stdout
    setvbuf(stdout, nullptr, _IONBF, 0);

    //int sample_rate = 11025;
    int sample_rate = 48000;
    bool only_sitor_b = false;
    bool reverse = false;
    navtex_rx nv(sample_rate, only_sitor_b, reverse, stdout, nullptr);

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
