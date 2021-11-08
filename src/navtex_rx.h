/* -*- c++ -*- */
/*
 * Copyright 2020 Franco Venturi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef _NAVTEX_RX_H
#define _NAVTEX_RX_H

#include "fftfilt.h"
#include <cstdio>
#include <vector>


class ccir_message : public std::string {
public:
    typedef std::pair<bool, ccir_message> detect_result;
    ccir_message();
    ccir_message(const std::string & s, char origin, char subject, int number);
    void reset_msg();
    detect_result detect_header();
    bool detect_end();
    void display(const std::string & alt_string);

private:
    static const size_t header_len = 10;
    static const size_t trunc_len = 5;

    // Header structure is:
    // ZCZCabcd message text NNNN
    // a  : Origin of the station.
    // b  : Message type.
    // cd : Message number from this station.
    char m_origin;
    char m_subject;
    int  m_number;

    // methods
    void cleanup();
    void init_members();
}; // ccir_message


class CCIR476 {
public:
    CCIR476();
    void char_to_code(std::string & str, int ch, bool & ex_shift) const;
    int code_to_char(int code, bool shift) const;
    int bytes_to_code(int * pos);
    int bytes_to_char(int * pos, int shift);
    static bool check_bits(int v);
    bool valid_char_at(int * pos);

private:
    unsigned char m_ltrs_to_code[128];
    unsigned char m_figs_to_code[128];
    bool m_valid_codes[128];
}; // CCIR476


class navtex_rx {
public:
    navtex_rx(int sample_rate, bool only_sitor_b, bool reverse, FILE* out);
    void process_data(const double * data, int nb_samples);

private:
    int m_sample_rate;
    bool m_only_sitor_b;
    bool m_reverse;
    FILE * m_out;

    // filter method related
    double m_center_frequency_f;

    double m_baud_rate;

    double m_mark_f;
    double m_space_f;
    double m_mark_phase;
    double m_space_phase;

    fftfilt *m_mark_lowpass;
    fftfilt *m_space_lowpass;

    double m_time_sec;
    double m_message_time;

    bool m_header_found;

    ccir_message m_curr_msg;

    int m_sample_count;
    double m_bit_sample_count;

    double m_early_accumulator;
    double m_prompt_accumulator;
    double m_late_accumulator;

    double m_next_early_event;
    double m_next_prompt_event;
    double m_next_late_event;
    double m_average_early_signal;
    double m_average_prompt_signal;
    double m_average_late_signal;

    bool m_pulse_edge_event;

    int m_averaged_mark_state;

    enum State { SYNC_SETUP, SYNC, READ_DATA };
    State m_state;

    int m_error_count;

    bool m_shift;

    bool m_alpha_phase;

    std::vector<int> m_bit_values;
    int m_bit_cursor;

    CCIR476 m_ccir476;


    // methods
    void set_filter_values();
    void configure_filters();
    void process_timeout();
    void flush_message(const std::string & extra_info);
    void display_message(ccir_message & ccir_msg, const std::string & alt_string );
    void put_received_message(const std::string & message);
    cmplx mixer(double & phase, double f, cmplx in);
    void process_fft_output(cmplx * zp_mark, cmplx * zp_space, int samples);
    void process_multicorrelator();
    double envelope_decay(double avg, double value);
    double noise_decay(double avg, double value);
    static const char * state_to_str(State s);
    void set_state(State s);
    void handle_bit_value(int accumulator);
    int find_alpha_characters();
    int process_bytes(int m_bit_cursor);
    bool process_char(int chr);
    void filter_print(int c);
    void put_rx_char(int c);
    void process_messages(int c);
}; // navtex_rx

#endif /* _NAVTEX_RX_H */
