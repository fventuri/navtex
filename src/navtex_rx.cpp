/* -*- c++ -*- */
/*
 * Copyright 2020 Franco Venturi.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// ---------------------------------------------------------------------
//
//  navtex.cxx
//
// Copyright (C) 2011-2016
//      Remi Chateauneu, F4ECW
//      Rik van Riel, AB1KW, <riel@surriel.com>
//
// This file is part of fldigi.  Adapted from code contained in JNX
// source code distribution.
//  JNX Copyright (C) Paul Lutus
// http://www.arachnoid.com/JNX/index.html
//
// fldigi is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// fldigi is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// Sync using multicorrelator, instead of null crossings
//      Rik van Riel, AB1KW, <riel@surriel.com>
//
// Null crossings are somewhat noisy, and the code to keep the navtex
// decoder in sync with the incoming signal using null crossings was
// rather fragile.
//
// Use a multicorrelator instead, which relies on the averaged magnitude
// of the signal accumulator to sync the decoder with the incoming signal.
//
// When debugging the code, the multicorrelator mostly corrects the
// modem forward in time, which can be explained by the fact that a
// bit takes 110.25 samples, while the code uses 110. When the NAVTEX
// transmitter is running at exactly 100 baud, one can expect to see
// the decoder get adjusted 25 times a second, to make up for the
// difference between 11000 and 11025.
//
// When multiple signals are on the air simultaneously, the null crossing
// code would often lose track of the signal. The multicorrelator seems
// to be more stable in this situation, though of course when both signals
// are close in strength things do not get decoded right.
//
// The signal sampling spread of 1/6 of the width of a bit was set through
// trial and error. A larger spread increases the signal difference between
// early, prompt, and late samples, but reduces the accumulator value seen
// by the demodulator. A smaller spread increases the accumulator value seen,
// but makes it harder to lock on in noisy conditions.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// low pass mark & space individually
//      Rik van Riel, AB1KW, <riel@surriel.com>
//
// Putting individual low pass filters on mark and space seems to
// result in an improved ability to overcome pulse noise, and decode
// weaker navtex signals.
//
// I have not found any signal where the performance of the codec
// got worse with this change.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// Correct display metric
//      Rik van Riel, AB1KW, <riel@surriel.com>
//
// The NAVTEX display_metric() function was buggy, in that decayavg
// returns the decayed value, but does not store it. It always put
// the current value in the metric, and kept avg_ratio at 0.0.
//
// This resulted in a somewhat chaotic, and not very useful metric
// display. Copy over the S/N calculation from the RTTY code, because
// that code seems to work well.

// Also print the S/N in Status2, like the RTTY code and other modes
// do.

// Copying over the RTTY S/N code wholesale might still not be
// enough, since the NAVTEX wave form appears to be somewhat
// different from RTTY.  However, at least we have something
// now, and the metric used for squelch seems to work again.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// Correct display metric
//      Rik van Riel, AB1KW, <riel@surriel.com>
//
// Widen afc filter for 'jump 90 Hz' code
//
// When the NAVTEX code spots a power imbalance of more than a factor
// 5 between mark and space, it will shift the frequency by 90 Hz.
// This is reported to help with some signals.
//
// However, it breaks with some other signals, which have a different
// spectral distribution between mark and space, with a spectrum looking
// something like this:
//
//                                        *
//                                        *
//                                        *
//                                       **
//     ******                           ***
//    ********                         ******
//   **********                       ********
//  ********************************************
// **********************************************
//
// In this spectrum, mark & space have a similar amount of energy,
// but that is only apparent when the comparison between them is
// done on a wider sample than 10 Hz.
//
// Sampling 30 Hz instead seems to result in a more stable AFC.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// use exact bit length
//      Rik van Riel, AB1KW, <riel@surriel.com>
//
// With a baud rate of 100 and a sample rate of 11025, the number
// of bits per sample is 110.25.  Approximating this with 110 bits
// per sample results in the decoder continuously chasing after the
// signal, and losing it more easily during transient noise or
// interference events.
//
// Simply changing the variable type from int to double makes life
// a little easier on the bit tracking code.
//
// The accumulator does not seem to care that it gets an extra sample
// every 4 bit periods.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// improvements to the multi correlator
//      Rik van Riel, AB1KW, <riel@surriel.com>
//
// While the multi correlator for bit sync was a nice improvement over
// the null crossing tracking, it did lose sync too easily in the presence
// of transient noise or interference, and was full of magic adjustments.
//
// Replace the magic adjustments with a calculation, which makes the multi
// correlator able to ride out transient noise or interference, and then
// make a larger adjustment all at once (if needed).
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// use same mark/space detector as RTTY modem
//      Rik van Riel, AB1KW, <riel@surriel.com>
//
// Switch the NAVTEX modem over to the same mark/spac decoder, with W7AY's
// automatic threshold correction algorithm, like the RTTY modem uses.
//
// The noise subtraction is a little different than in the RTTY modem;
// the algorithm used in W7AY's code seems to work a little better with
// the noise present at 518 kHz, when compared to the algorithm used in
// the RTTY modem.
//
// I have compared this detector to a correlation detector; the latter
// appears to be a little more sensitive, which includes higher
// sensitivity to noise. With a 250 Hz filter on the radio, the
// correlation detector might be a little bit better, while with the
// filter on the radio opened up to 4kHz wide, this detector appears
// to be more robust.
//
// On signals with a large mark/space power imbalance, or where the power
// distribution in one of the two throws off the automatic frequency
// correction, this decoder is able to handle signals that neither of
// the alternatives tested does.
// ---------------------------------------------------------------------

#include "misc.h"
#include "navtex_rx.h"
#include <climits>
#include <cstring>

static const int deviation_f = 85;
static const double dflt_center_freq = 1000.0 ;

// Minimum length of logged messages
static const size_t min_siz_logged_msg = 0;

// LOG levels and macros
enum LogLevel {
    DEBUG, INFO, WARN
};

static const LogLevel log_level = WARN;

#define LOG_DEBUG(...) if (log_level <= DEBUG) { fprintf(stderr, "[DEBUG] "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); }
#define LOG_INFO(...) if (log_level <= INFO) { fprintf(stderr, "[INFO] "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); }
#define LOG_WARN(...) if (log_level <= WARN) { fprintf(stderr, "[WARN] "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); }


navtex_rx::navtex_rx(int sample_rate, bool only_sitor_b, bool reverse, FILE * out) {
    m_sample_rate = sample_rate;
    m_only_sitor_b = only_sitor_b;
    m_reverse = reverse;
    m_out = out;

    m_center_frequency_f = dflt_center_freq;
    // this value must never be zero and bigger than 10.
    m_baud_rate = 100;
    double m_bit_duration_seconds = 1.0 / m_baud_rate;
    m_bit_sample_count = m_sample_rate * m_bit_duration_seconds;

    m_time_sec = 0.0;
    m_message_time = 0.0;

    m_header_found = false;

    m_sample_count = 0;

    m_early_accumulator = 0;
    m_prompt_accumulator = 0;
    m_late_accumulator = 0;

    // A narrower spread between signals allows the modem to
    // center on the pulses better, but a wider spread makes
    // more robust under noisy conditions. 1/5 seems to work.
    m_next_early_event = 0;
    m_next_prompt_event = m_bit_sample_count / 5;
    m_next_late_event = m_bit_sample_count * 2 / 5;
    m_average_early_signal = 0;
    m_average_prompt_signal = 0;
    m_average_late_signal = 0;

    m_pulse_edge_event = false;
    m_averaged_mark_state = 0;

    m_state = SYNC_SETUP;

    m_error_count = 0;

    m_shift = false;

    m_alpha_phase = false;

    // keep 1 second worth of bit values for decoding
    m_bit_values.resize(m_baud_rate);
    m_bit_cursor = 0;

    m_mark_lowpass = 0;
    m_space_lowpass = 0;

    set_filter_values();
    configure_filters();
}

void navtex_rx::process_data(const double * data, int nb_samples) {

    cmplx z, zmark, zspace, *zp_mark, *zp_space;

    process_timeout();

    for (int i = 0; i < nb_samples; i++) {
        int n_out;

        m_time_sec = m_sample_count / m_sample_rate ;

        double dv = 32767 * data[i];
        z = cmplx(dv, dv);

        zmark = mixer(m_mark_phase, m_mark_f, z);
        m_mark_lowpass->run(zmark, &zp_mark);

        zspace = mixer(m_space_phase, m_space_f, z);
        n_out = m_space_lowpass->run(zspace, &zp_space);

        if (n_out)
            process_fft_output(zp_mark, zp_space, n_out);
    }
}


// private functions
void navtex_rx::set_filter_values() {
    m_mark_f = m_center_frequency_f + deviation_f;
    m_space_f = m_center_frequency_f - deviation_f;
    m_mark_phase = 0;
    m_space_phase = 0;
}

void navtex_rx::configure_filters() {
    const int filtlen = 512;
    if (m_mark_lowpass) delete m_mark_lowpass;
    m_mark_lowpass = new fftfilt(m_baud_rate/m_sample_rate, filtlen);
    m_mark_lowpass->rtty_filter(m_baud_rate/m_sample_rate);

    if (m_space_lowpass) delete m_space_lowpass;
    m_space_lowpass = new fftfilt(m_baud_rate/m_sample_rate, filtlen);
    m_space_lowpass->rtty_filter(m_baud_rate/m_sample_rate);
}

// Checks that we have no waited too long, and if so, flushes the message with a specific terminator.
void navtex_rx::process_timeout() {
    // No messaging in SitorB
    if (m_only_sitor_b) return;

    bool timeOut = m_time_sec - m_message_time > 600;
    if (!timeOut) return;
    LOG_INFO("Timeout: time_sec=%lf, message_time=%lf", m_time_sec, m_message_time );

    // TODO: Headerless messages could be dropped if shorter than X chars.
    flush_message(":<TIMEOUT>");
}

// The parameter is appended at the message end.
void navtex_rx::flush_message(const std::string & extra_info)
{
    if (m_header_found)
    {
        m_header_found = false;
        display_message(m_curr_msg, m_curr_msg + extra_info);
    }
    else
    {
        display_message(m_curr_msg, "[Lost header]:" + m_curr_msg + extra_info);
    }
    m_curr_msg.reset_msg();
    m_message_time = m_time_sec;
}

void navtex_rx::display_message(ccir_message & ccir_msg, const std::string & alt_string)
{
    if (ccir_msg.size() >= min_siz_logged_msg) {
        try {
            ccir_msg.display(alt_string);
            put_received_message(alt_string);
        } catch (const std::exception & exc) {
            LOG_WARN("Caught %s", exc.what());
        }
    }
    else
    {
        LOG_INFO("Do not log short message:%s", ccir_msg.c_str() );
    }
}

// Called by the engine each time a message is saved.
void navtex_rx::put_received_message(const std::string & message)
{
    LOG_INFO("%s", message.c_str());
    fputs(message.c_str(), m_out);
}

cmplx navtex_rx::mixer(double & phase, double f, cmplx in)
{
    cmplx z = cmplx( cos(phase), sin(phase)) * in;

    phase -= 2.0 * M_PI * f / m_sample_rate;
    if (phase < -2.0 * M_PI) phase += 2.0 * M_PI;

    return z;
}

void navtex_rx::process_fft_output(cmplx * zp_mark, cmplx * zp_space, int samples)
{
    // envelope & noise levels for mark & space, respectively
    static double mark_env = 0, space_env = 0;
    static double mark_noise = 0, space_noise = 0;

    for (int i = 0; i < samples; i++) {
        double mark_abs = abs(zp_mark[i]);
        double space_abs = abs(zp_space[i]);

        process_multicorrelator();

        // determine noise floor & envelope for mark & space
        mark_env = envelope_decay(mark_env, mark_abs);
        mark_noise = noise_decay(mark_noise, mark_abs);

        space_env = envelope_decay(space_env, space_abs);
        space_noise = noise_decay(space_noise, space_abs);

        double noise_floor = (space_noise + mark_noise) / 2;

        // clip mark & space to envelope & floor
        mark_abs = std::min(mark_abs, mark_env);
        mark_abs = std::max(mark_abs, noise_floor);

        space_abs = std::min(space_abs, space_env);
        space_abs = std::max(space_abs, noise_floor);

        // mark-space discriminator with automatic threshold
        // correction, see:
        // http://www.w7ay.net/site/Technical/ATC/
        double logic_level =
            (mark_abs - noise_floor) * (mark_env - noise_floor) -
            (space_abs - noise_floor) * (space_env - noise_floor) -
            0.5 * ( (mark_env - noise_floor) * (mark_env - noise_floor) -
                 (space_env - noise_floor) * (space_env - noise_floor));

        // Using the logarithm of the logic_level tells the
        // bit synchronization and character decoding which
        // samples were decoded well, and which poorly.
        // This helps fish signals out of the noise.
        int mark_state = log(1 + abs(logic_level));
        if (logic_level < 0)
            mark_state = -mark_state;
        m_early_accumulator += mark_state;
        m_prompt_accumulator += mark_state;
        m_late_accumulator += mark_state;

        // An average of the magnitude of the accumulator
        // is taken at the sample point, as well as a quarter
        // bit before and after. This allows the code to see
        // the best time to sample the signal without relying
        // on (noisy) null crossings.
        if (m_sample_count >= m_next_early_event) {
            m_average_early_signal = decayavg(
                    m_average_early_signal,
                    fabs(m_early_accumulator), 64);
            m_next_early_event += m_bit_sample_count;
            m_early_accumulator = 0;
        }

        if (m_sample_count >= m_next_late_event) {
            m_average_late_signal = decayavg(
                    m_average_late_signal,
                    fabs(m_late_accumulator), 64);
            m_next_late_event += m_bit_sample_count;
            m_late_accumulator = 0;
        }

        // the end of a signal pulse
        // the accumulator should be at maximum deviation
        m_pulse_edge_event = m_sample_count >= m_next_prompt_event;
        if (m_pulse_edge_event) {
            m_average_prompt_signal = decayavg(
                    m_average_prompt_signal,
                    fabs(m_prompt_accumulator), 64);
            m_next_prompt_event += m_bit_sample_count;
            m_averaged_mark_state = m_prompt_accumulator;
            if (m_reverse)
                m_averaged_mark_state = -m_averaged_mark_state;
            m_prompt_accumulator = 0;
        }

        switch (m_state) {
            case SYNC_SETUP:
                m_error_count = 0;
                m_shift = false;
                set_state(SYNC);
                break;
            case SYNC:
            case READ_DATA:
                if (m_pulse_edge_event)
                    handle_bit_value(m_averaged_mark_state);
        }

        m_sample_count++;
    }
}

// The signal is sampled at three points: early, prompt, and late.
// The prompt event is where the signal is decoded, while early and
// late are only used to adjust the time of the sampling to match
// the incoming signal.
//
// The early event happens 1/5 bit period before the prompt event,
// and the late event 1/5 bit period later. If the incoming signal
// peaks early, it means the decoder is late. That is, if the early
// signal is "too large", decoding should to happen earlier.
//
// Attempt to center the signal so the accumulator is at its
// maximum deviation at the prompt event. If the bit is decoded
// too early or too late, the code is more sensitive to noise,
// and less likely to decode the signal correctly.
void navtex_rx::process_multicorrelator()
{
    // Adjust the sampling period once every 8 bit periods.
    if (m_sample_count % (int)(m_bit_sample_count * 8))
        return;

    // Calculate the slope between early and late signals
    // to align the logic sampling with the received signal
    double slope = m_average_late_signal - m_average_early_signal;

    if (m_average_prompt_signal * 1.05 < m_average_early_signal &&
        m_average_prompt_signal * 1.05 < m_average_late_signal) {
        // At a signal minimum. Get out quickly.
        if (m_average_early_signal > m_average_late_signal) {
            // move prompt to where early is
            slope = m_next_early_event - m_next_prompt_event;
            slope = fmod(slope - m_bit_sample_count, m_bit_sample_count);
            m_average_late_signal = m_average_prompt_signal;
            m_average_prompt_signal = m_average_early_signal;
        } else {
            // move prompt to where late is
            slope = m_next_late_event - m_next_prompt_event;
            slope = fmod(slope + m_bit_sample_count, m_bit_sample_count);
            m_average_early_signal = m_average_prompt_signal;
            m_average_prompt_signal = m_average_late_signal;
        }
    } else
        slope /= 1024;

    if (slope) {
        m_next_early_event += slope;
        m_next_prompt_event += slope;
        m_next_late_event += slope;
        LOG_DEBUG("adjusting by %1.2f, early %1.1f, prompt %1.1f, late %1.1f", slope, m_average_early_signal, m_average_prompt_signal, m_average_late_signal);
    }
}

// envelope average decays fast up, slow down
double navtex_rx::envelope_decay(double avg, double value) {
    int divisor;
    if (value > avg)
        divisor = m_bit_sample_count / 4;
    else
        divisor = m_bit_sample_count * 16;
    return decayavg(avg, value, divisor);
}

// noise average decays fast down, slow up
double navtex_rx::noise_decay(double avg, double value) {
    int divisor;
    if (value < avg)
        divisor = m_bit_sample_count / 4;
    else
        divisor = m_bit_sample_count * 48;
    return decayavg(avg, value, divisor);
}

const char * navtex_rx::state_to_str(State s) {
    switch(s) {
        case SYNC_SETUP: return "SYNC_SETUP";
        case SYNC      : return "SYNC";
        case READ_DATA : return "READ_DATA";
        default        : return "Unknown" ;
    }
}

void navtex_rx::set_state(State s) {
    if (s != m_state) {
        m_state = s;
        LOG_INFO("State: %s", state_to_str(m_state));
    }
}

// Turns accumulator values (estimates of whether a bit is 1 or 0)
// into navtex messages
void navtex_rx::handle_bit_value(int accumulator) {
    int buffersize = m_bit_values.size();
    int i, offset = 0;

    // Store the received value in the bit stream
    for (i = 0; i < buffersize - 1; i++) {
        m_bit_values[i] = m_bit_values[i+1];
    }
    m_bit_values[buffersize - 1] = accumulator;
    if (m_bit_cursor > 0)
        m_bit_cursor--;

    // Find the most likely location where the message starts
    if (m_state == SYNC) {
        offset = find_alpha_characters();
        if (offset >= 0) {
            set_state(READ_DATA);
            m_bit_cursor = offset;
            m_alpha_phase = true;
        } else
            set_state(SYNC_SETUP);
    }

    // Process 7-bit characters as they come in,
    // skipping rep (duplicate) characters
    if (m_state == READ_DATA) {
        if (m_bit_cursor < buffersize - 7) {
            if (m_alpha_phase) {
                int ret = process_bytes(m_bit_cursor);
                m_error_count -= ret;
                if (m_error_count > 5)
                    set_state(SYNC_SETUP);
                if (m_error_count < 0)
                    m_error_count = 0;
            }
            m_alpha_phase = !m_alpha_phase;
            m_bit_cursor += 7;
        }
    }
}

static const int code_ltrs = 0x5a;
static const int code_figs = 0x36;
static const int code_alpha = 0x0f;
static const int code_beta = 0x33;
static const int code_char32 = 0x6a;
static const int code_rep = 0x66;
static const int char_bell = 0x07;

static int fec_offset(int offset);

// Try to find a position in the bit stream with:
// - the largest number of valid characters, and
// - with rep (duplicate) characters in the right locations
// This way the code can sync up with an incoming signal after
// the initial alpha/rep synchronisation
//
// http://www.arachnoid.com/JNX/index.html
// "NAUTICAL" becomes:
// rep alpha rep alpha N alpha A alpha U N T A I U C T A I L C blank A blank L
int navtex_rx::find_alpha_characters() {
    int best_offset = 0;
    int best_score = 0;
    int offset, i;

    // With 7 bits per character, and interleaved rep & alpha
    // characters, the first alpha character with a corresponding
    // rep in the stream can be in any of 14 locations
    for (offset = 35; offset < (35 + 14); offset++) {
        int score = 0;
        int reps = 0;
        int limit = m_bit_values.size() - 7;

        // Search for the largest sequence of valid characters
        for (i = offset; i < limit; i += 7) {
            if (m_ccir476.valid_char_at(&m_bit_values[i])) {
                int ri = fec_offset(i);
                int code = m_ccir476.bytes_to_code(&m_bit_values[i]);
                int rep = m_ccir476.bytes_to_code(&m_bit_values[ri]);

                // This character is valid
                score++;

                // Does it match its rep?
                if (code == rep) {
                    // This offset is wrong, rep
                    // and alpha are spaced odd
                    if (code == code_alpha ||
                        code == code_rep) {
                        score = 0;
                        continue;
                    }
                    reps++;
                } else if (code == code_alpha) {
                    // Is there a matching rep to
                    // this alpha?
                    int ri = i - 7;
                    int rep = m_ccir476.bytes_to_code(&m_bit_values[ri]);
                    if (rep == code_rep) {
                        reps++;
                    }
                }
            }
        }

        // the most valid characters, with at least 3 FEC reps
        if (reps >= 3 && score + reps > best_score) {
            best_score = score + reps;
            best_offset = offset;
        }
    }

    // m_bit_values fits 14 characters; if there are at least
    // 9 good ones, tell the caller where they start
    if (best_score > 8)
        return best_offset;
    else
        return -1;
}

static void flip_smallest_bit(int * pos);

// Turn a series of 7 bit confidence values into a character
//
// 1 on successful decode of the alpha character
// 0 on unmodified FEC replacement
// -1 on soft failure (FEC calculation)
// -2 on hard failure
int navtex_rx::process_bytes(int m_bit_cursor) {
    int code = m_ccir476.bytes_to_code(&m_bit_values[m_bit_cursor]);
    int success = 0;

    if (m_ccir476.check_bits(code)) {
        LOG_DEBUG("valid code : %x (%c)", code, m_ccir476.code_to_char(code, m_shift));
        success = 1;
        goto decode;
    }

    if (fec_offset(m_bit_cursor) < 0)
        return -1;

    // The alpha (primary) character received was not correct.
    // Try the rep (duplicate) copy of the character, and some
    // permutations to see if the correct character can be found.
    {
        int i, calc, avg[7];
        // Rep is 5 characters before alpha.
        int reppos = fec_offset(m_bit_cursor);
        int rep = m_ccir476.bytes_to_code(&m_bit_values[reppos]);
        if (CCIR476::check_bits(rep)) {
            // Current code is probably code_alpha.
            // Skip decoding to avoid switching phase.
            if (rep == code_rep)
                return 0;
            LOG_DEBUG("FEC replacement: %x -> %x (%c)", code, rep, m_ccir476.code_to_char(rep, m_shift));
            code = rep;
            goto decode;
        }

        // Neither alpha or rep are valid. Check whether
        // the average of the two is a valid character.
        for (i = 0; i < 7; i++) {
            int a = m_bit_values[m_bit_cursor + i];
            int r = m_bit_values[reppos + i];
            avg[i] = a + r;
        }

        calc = m_ccir476.bytes_to_code(avg);
        if (CCIR476::check_bits(calc)) {
            LOG_DEBUG("FEC calculation: %x & %x -> %x (%c)", code, rep, calc, m_ccir476.code_to_char(calc, m_shift));
            code = calc;
            success = -1;
            goto decode;
        }

        // Flip the lowest confidence bit in alpha.
        flip_smallest_bit(&m_bit_values[m_bit_cursor]);
        calc = m_ccir476.bytes_to_code(&m_bit_values[m_bit_cursor]);
        if (CCIR476::check_bits(calc)) {
            LOG_DEBUG("FEC calculation: %x & %x -> %x (%c)", code, rep, calc, m_ccir476.code_to_char(calc, m_shift));
            code = calc;
            success = -1;
            goto decode;
        }

        // Flip the lowest confidence bit in rep.
        flip_smallest_bit(&m_bit_values[reppos]);
        calc = m_ccir476.bytes_to_code(&m_bit_values[reppos]);
        if (CCIR476::check_bits(calc)) {
            LOG_DEBUG("FEC calculation: %x & %x -> %x (%c)", code, rep, calc, m_ccir476.code_to_char(calc, m_shift));
            code = calc;
            success = -1;
            goto decode;
        }

        // Try flipping the bit with the lowest confidence
        // in the average of alpha & rep.
        flip_smallest_bit(avg);
        calc = m_ccir476.bytes_to_code(avg);
        if (CCIR476::check_bits(calc)) {
            LOG_DEBUG("FEC calculation: %x & %x -> %x (%c)", code, rep, calc, m_ccir476.code_to_char(calc, m_shift));
            code = calc;
            success = -1;
            goto decode;
        }

        LOG_DEBUG("decode fail %x, %x", code, rep);
        return -2;
    }

decode:
    process_char(code);
    return success;
}

bool navtex_rx::process_char(int chr) {
    static int last_char = 0;
    switch (chr) {
        case code_rep:
            // This code should run in alpha phase, but
            // it just received two rep characters. Fix
            // the rep/alpha phase, so FEC works again.
            if (last_char == code_rep) {
                LOG_DEBUG("fixing rep/alpha sync");
                m_alpha_phase = false;
            }
            break;
        case code_alpha:
            break;
        case code_beta:
            break;
        case code_char32:
            break;
        case code_ltrs:
            m_shift = false;
            break;
        case code_figs:
            m_shift = true;
            break;
        default:
            chr = m_ccir476.code_to_char(chr, m_shift);
            if (chr < 0) {
                LOG_INFO("Missed this code: %x", abs(chr));
            } else {
                filter_print(chr);
                process_messages(chr);
            }
            break;
        } // switch

    last_char = chr;
    return true;
}

void navtex_rx::filter_print(int c) {
    if (c == char_bell) {
        /// TODO: It should be a beep, but French navtex displays a quote.
        put_rx_char('\'');
    } else if (c != -1 && c != '\r' && c != code_alpha && c != code_rep) {
        put_rx_char(c);
    }
}

void navtex_rx::put_rx_char(int c) {
    // fv - TODO
    putc(c, m_out);
}

void navtex_rx::process_messages(int c) {
    m_curr_msg.push_back((char) c);

    /// No header nor trailer for plain SitorB.
    if ( m_only_sitor_b ) {
        m_header_found = true;
        m_message_time = m_time_sec;
        return;
    }

    ccir_message::detect_result msg_cut = m_curr_msg.detect_header();
    if ( msg_cut.first ) {
        /// Maybe the message was already valid.
        if( m_header_found )
        {
            display_message( msg_cut.second, msg_cut.second + ":[Lost trailer]" );
        }
        else
        {
            /// Maybe only non-significant chars.
            if( ! msg_cut.second.empty() )
            {
                display_message( msg_cut.second, "[Lost header]:" + msg_cut.second + ":[Lost trailer]" );
            }
        }
        m_header_found = true;
        m_message_time = m_time_sec;

    } else { // valid message state
        if ( m_curr_msg.detect_end() ) {
            flush_message("");
        }
    }
}

// The rep character is transmitted 5 characters (35 bits) ahead of
// the alpha character.
static int fec_offset(int offset) {
    return offset - 35;
}

// Flip the sign of the smallest (least certain) bit in a character;
// hopefully this will result in the right valid character.
static void flip_smallest_bit(int * pos) {
    int min_zero = INT_MIN, min_one = INT_MAX;
    int min_zero_pos = -1, min_one_pos = -1;
    int count_zero = 0, count_one = 1;
    int val, i;

    for (i = 0; i < 7; i++) {
        val = pos[i];
        if (val < 0) {
            count_zero++;
            if (val > min_zero) {
                min_zero = val;
                min_zero_pos = i;
            }
        } else {
            count_one++;
            if (val < min_one) {
                min_one = val;
                min_one_pos = i;
            }
        }
    }

    // A valid character has 3 zeroes and 4 ones, if we have
    // 5 ones or 4 zeroes, flipping the smallest one would make
    // this character valid.
    if (count_zero == 4)
        pos[min_zero_pos] = -pos[min_zero_pos];
    else if (count_one == 5)
        pos[min_one_pos] = -pos[min_one_pos];
}


// ccir_message
ccir_message::ccir_message() {
    init_members();
}

ccir_message::ccir_message(const std::string & s, char origin, char subject, int number) :
    std::string(s),
    m_origin(origin),
    m_subject(subject),
    m_number(number) {
    cleanup();
}

void ccir_message::reset_msg() {
    clear();
    init_members();
}

ccir_message::detect_result ccir_message::detect_header() {
    size_t qlen = size();

    if (qlen >= header_len) {
        const char * comp = & (*this)[qlen - header_len];
        if (
            (comp[0] == 'Z') &&
            (comp[1] == 'C') &&
            (comp[2] == 'Z') &&
            (comp[3] == 'C') &&
            (comp[4] == ' ') &&
            isalnum(comp[5]) &&
            isalnum(comp[6]) &&
            isdigit(comp[7]) &&
            isdigit(comp[8]) &&
            // (comp[9] == '\r') )
            (strchr("\n\r", comp[9]))) {

            // This returns the garbage before the valid header.
            // Garbage because the trailer could not be read, but maybe header OK.
            ccir_message msg_cut(substr(0, size() - header_len),
                                 m_origin, m_subject, m_number);
            m_origin  = comp[5];
            m_subject = comp[6];
            m_number = (comp[7] - '0') * 10 + (comp[8] - '0');
            // Remove the beginning useless chars.
            /// TODO: Read broken headers such as "ZCZC EA0?"
            clear();
            return detect_result(true, msg_cut);
        }
    }
    return detect_result(false, ccir_message());
}

bool ccir_message::detect_end() {
    // Should be "\r\nNNNN\r\n" theoretically, but tolerates shorter strings.
    static const size_t slen = 4;
    static const char stop_valid[slen + 1] = "NNNN";
    size_t qlen = size();
    if (qlen < slen) {
        return false;
    }
    std::string comp = substr(qlen - slen, slen);
    bool end_seen = comp == stop_valid;
    if(end_seen) {
        erase(qlen - slen, slen);
        LOG_INFO("\n%s", c_str());
    }
    return end_seen;
}

void ccir_message::display(const std::string & alt_string) {
    std::string::operator=(alt_string);
    cleanup();
}


// private functions for ccir_message

// Remove non-Ascii chars, replace new-line by special character etc....
void ccir_message::cleanup() {
    // This is temporary, to manipulate a multi-line string.
    static const char * new_line = "\n";

    // It would be possible to do the change in place, because the new string
    // it shorter than the current one, but at the expense of clarity.
    bool wasDelim = false, wasSpace = false, chrSeen = false;
    std::string newStr ;
    for (iterator it = begin(); it != end(); ++it) {
        switch(*it) {
            case '\n':
            case '\r': wasDelim = true ;
                       break ;
            case ' ':
            case '\t': wasSpace = true ;
                       break ;
            default: if (chrSeen) {
                         if (wasDelim) {
                             newStr.append(new_line);
                         } else if(wasSpace) {
                             newStr.push_back(' ');
                         }
                     }
                     wasDelim = false;
                     wasSpace = false;
                     chrSeen = true;
                     newStr.push_back(*it);
        }
    }
    swap(newStr);
}

void ccir_message::init_members() {
    m_origin = '?';
    m_subject = '?';
    m_number = 0;
}


// CCIR476
static const unsigned char code_to_ltrs[128] = {
    //0 1   2   3   4   5   6   7   8   9   a   b   c   d   e   f
    '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', // 0
    '_', '_', '_', '_', '_', '_', '_', 'J', '_', '_', '_', 'F', '_', 'C', 'K', '_', // 1
    '_', '_', '_', '_', '_', '_', '_', 'W', '_', '_', '_', 'Y', '_', 'P', 'Q', '_', // 2
    '_', '_', '_', '_', '_', 'G', '_', '_', '_', 'M', 'X', '_', 'V', '_', '_', '_', // 3
    '_', '_', '_', '_', '_', '_', '_', 'A', '_', '_', '_', 'S', '_', 'I', 'U', '_', // 4
    '_', '_', '_', 'D', '_', 'R', 'E', '_', '_', 'N', '_', '_', ' ', '_', '_', '_', // 5
    '_', '_', '_', 'Z', '_', 'L', '_', '_', '_', 'H', '_', '_', '\n', '_', '_', '_', // 6
    '_', 'O', 'B', '_', 'T', '_', '_', '_', '\r', '_', '_', '_', '_', '_', '_', '_' // 7
};

static const unsigned char code_to_figs[128] = {
    //0 1   2   3   4   5   6   7   8   9   a   b   c   d   e   f
    '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '_', // 0
    '_', '_', '_', '_', '_', '_', '_', '\'', '_', '_', '_', '!', '_', ':', '(', '_', // 1
    '_', '_', '_', '_', '_', '_', '_', '2', '_', '_', '_', '6', '_', '0', '1', '_', // 2
    '_', '_', '_', '_', '_', '&', '_', '_', '_', '.', '/', '_', ';', '_', '_', '_', // 3
    '_', '_', '_', '_', '_', '_', '_', '-', '_', '_', '_', '\07', '_', '8', '7', '_', // 4
    '_', '_', '_', '$', '_', '4', '3', '_', '_', ',', '_', '_', ' ', '_', '_', '_', // 5
    '_', '_', '_', '"', '_', ')', '_', '_', '_', '#', '_', '_', '\n', '_', '_', '_', // 6
    '_', '9', '?', '_', '5', '_', '_', '_', '\r', '_', '_', '_', '_', '_', '_', '_' // 7
};

CCIR476::CCIR476() {
    memset(m_ltrs_to_code, 0, 128);
    memset(m_figs_to_code, 0, 128);
    for (size_t i = 0; i < 128; i++) m_valid_codes[i] = false ;
    for (int code = 0; code < 128; code++) {
        // Valid codes have four bits set only. This leaves three bits for error detection.
        // TODO: If a code is invalid, we could take the closest value in terms of bits.
        if (check_bits(code)) {
            m_valid_codes[code] = true;
            unsigned char figv = code_to_figs[code];
            unsigned char ltrv = code_to_ltrs[code];
            if (figv != '_') {
                m_figs_to_code[figv] = code;
            }
            if (ltrv != '_') {
                m_ltrs_to_code[ltrv] = code;
            }
        }
    }
}

void CCIR476::char_to_code(std::string & str, int ch, bool & ex_shift) const {
    ch = toupper(ch);
    // avoid unnecessary shifts
    if (ex_shift && m_figs_to_code[ch] != '\0') {
        str.push_back(m_figs_to_code[ch]);
    }
    else if (!ex_shift && m_ltrs_to_code[ch] != '\0') {
        str.push_back(m_ltrs_to_code[ch]);
    }
    else if (m_figs_to_code[ch] != '\0') {
        ex_shift = true;
        str.push_back(code_figs);
        str.push_back(m_figs_to_code[ch]);
    }
    else if (m_ltrs_to_code[ch] != '\0') {
        ex_shift = false;
        str.push_back(code_ltrs);
        str.push_back(m_ltrs_to_code[ch]);
    }
}

int CCIR476::code_to_char(int code, bool shift) const {
    const unsigned char * target = (shift) ? code_to_figs : code_to_ltrs;
    if (target[code] != '_') {
        return target[code];
    }
    // default: return negated code
    return -code;
}

int CCIR476::bytes_to_code(int * pos) {
    int code = 0;
    int i;

    for (i = 0; i < 7; i++)
        code |= ((pos[i] > 0) << i);
    return code;
}

int CCIR476::bytes_to_char(int * pos, int shift) {
    int code = bytes_to_code(pos);
    return code_to_char(code, shift);
}

// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetNaive
/// Counting set bits, Brian Kernighan's way
bool CCIR476::check_bits(int v) {
    int bc = 0;
    while (v != 0) {
        bc++;
        v &= v - 1;
    }
    //printf("check_bits %d %d %c\n", bc, (int)code_to_ltrs[v], code_to_ltrs[v] );
    return bc == 4;
}

// Is there a valid character in the next 7 ints?
bool CCIR476::valid_char_at(int * pos) {
    int count = 0;
    int i;

    for (i = 0; i < 7; i++)
        if (pos[i] > 0)
            count++;

    return (count == 4);
}
