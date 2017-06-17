/*
   Copyright (C) 2007, 2008, 2009, 2010, 2011 Her Majesty the Queen in
   Right of Canada (Communications Research Center Canada)

   Copyright (C) 2017
   Matthias P. Braendli, matthias.braendli@mpb.li
   Andreas Steger, andreas.steger@digris.ch

    http://opendigitalradio.org

   This block implements a memoryless polynom for digital predistortion.
   For better performance, multiplying is done in another thread, leading
   to a pipeline delay of two calls to MemlessPoly::process
 */
/*
   This file is part of ODR-DabMod.

   ODR-DabMod is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as
   published by the Free Software Foundation, either version 3 of the
   License, or (at your option) any later version.

   ODR-DabMod is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with ODR-DabMod.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "MemlessPoly.h"
#include "PcDebug.h"
#include "Utils.h"

#include <stdio.h>
#include <stdexcept>

#include <array>
#include <iostream>
#include <fstream>
#include <memory>

using namespace std;


// By default the signal is unchanged
static const std::array<float, 16> default_coefficients({{
        1, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        1, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        }});


MemlessPoly::MemlessPoly(const std::string& coefs_file) :
    PipelinedModCodec(),
    RemoteControllable("memlesspoly"),
    m_coefs_file(coefs_file)
{
    PDEBUG("MemlessPoly::MemlessPoly(%s) @ %p\n",
            coefs_file.c_str(), this);

    RC_ADD_PARAMETER(ncoefs, "(Read-only) number of coefficients.");
    RC_ADD_PARAMETER(coeffile, "Filename containing coefficients. When written to, the new file gets automatically loaded.");

    load_coefficients(m_coefs_file);

    start_pipeline_thread();
}

void MemlessPoly::load_coefficients(const std::string &coefFile)
{
    std::vector<float> coefs;
    if (coefFile == "default") {
        std::copy(default_coefficients.begin(), default_coefficients.end(),
                std::back_inserter(coefs));
    }
    else {
        std::ifstream coef_fstream(coefFile.c_str());
        if(!coef_fstream) {
            fprintf(stderr, "MemlessPoly: file %s could not be opened !\n", coefFile.c_str());
            throw std::runtime_error("MemlessPoly: Could not open file with coefs! ");
        }
        int n_coefs;
        coef_fstream >> n_coefs;

        if (n_coefs <= 0) {
            fprintf(stderr, "MemlessPoly: warning: coefs file has invalid format\n");
            throw std::runtime_error("MemlessPoly: coefs file has invalid format.");
        }

        if (n_coefs != 16) {
            throw std::runtime_error( "MemlessPoly: error: coefs file does not have 16 coefs\n");
        }

        fprintf(stderr, "MemlessPoly: Reading %d coefs...\n", n_coefs);

        coefs.resize(n_coefs);

        int n;
        for (n = 0; n < n_coefs; n++) {
            coef_fstream >> coefs[n];
            PDEBUG("MemlessPoly: coef: %f\n",  coefs[n] );
            if (coef_fstream.eof()) {
                fprintf(stderr, "MemlessPoly: file %s should contains %d coefs, but EOF reached "\
                        "after %d coefs !\n", coefFile.c_str(), n_coefs, n);
                throw std::runtime_error("MemlessPoly: coefs file invalid ! ");
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_coefs_mutex);

        m_coefs = coefs;
    }
}


int MemlessPoly::internal_process(Buffer* const dataIn, Buffer* dataOut)
{
    dataOut->setLength(dataIn->getLength());

    const float* in = reinterpret_cast<const float*>(dataIn->getData());
    float* out = reinterpret_cast<float*>(dataOut->getData());
    size_t sizeOut = dataOut->getLength() / sizeof(float);

    {

         std::lock_guard<std::mutex> lock(m_coefs_mutex);
         for (size_t i = 0; i < sizeOut; i += 4) {
             float out_0 = out[i+0];
             float out_1 = out[i+1];
             float out_2 = out[i+2];
             float out_3 = out[i+3];

             float in_0 = in[i+0];
             float in_1 = in[i+1];
             float in_2 = in[i+2];
             float in_3 = in[i+3];

             float mag_0 = std::abs(in[i]);
             float mag_1 = std::abs(in[i+1]);
             float mag_2 = std::abs(in[i+2]);
             float mag_3 = std::abs(in[i+3]);

             float mag_0_1 = mag_0;
             float mag_0_2 = mag_0_1 * mag_0_1;
             float mag_0_3 = mag_0_2 * mag_0_1;
             float mag_0_4 = mag_0_2 * mag_0_2;
             float mag_0_5 = mag_0_3 * mag_0_2;
             float mag_0_6 = mag_0_3 * mag_0_3;
             float mag_0_7 = mag_0_4 * mag_0_3;

             float mag_1_1 = mag_1;
             float mag_1_2 = mag_1_1 * mag_1_1;
             float mag_1_3 = mag_1_2 * mag_1_1;
             float mag_1_4 = mag_1_2 * mag_1_2;
             float mag_1_5 = mag_1_3 * mag_1_2;
             float mag_1_6 = mag_1_3 * mag_1_3;
             float mag_1_7 = mag_1_4 * mag_1_3;

             float mag_2_1 = mag_2;
             float mag_2_2 = mag_2_1 * mag_2_1;
             float mag_2_3 = mag_2_2 * mag_2_1;
             float mag_2_4 = mag_2_2 * mag_2_2;
             float mag_2_5 = mag_2_3 * mag_2_2;
             float mag_2_6 = mag_2_3 * mag_2_3;
             float mag_2_7 = mag_2_4 * mag_2_3;

             float mag_3_1 = mag_3;
             float mag_3_2 = mag_3_1 * mag_3_1;
             float mag_3_3 = mag_3_2 * mag_3_1;
             float mag_3_4 = mag_3_2 * mag_3_2;
             float mag_3_5 = mag_3_3 * mag_3_2;
             float mag_3_6 = mag_3_3 * mag_3_3;
             float mag_3_7 = mag_3_4 * mag_3_3;

             out_0 = in_0 * (
                 m_coefs[0] +
                 m_coefs[2]  * mag_0_1 +
                 m_coefs[4]  * mag_0_2 +
                 m_coefs[6]  * mag_0_3 +
                 m_coefs[8]  * mag_0_4 +
                 m_coefs[10] * mag_0_5 +
                 m_coefs[12] * mag_0_6 +
                 m_coefs[14] * mag_0_7
                );

             out_1 = in_1 * (
                 m_coefs[1] +
                 m_coefs[3]  * mag_1_1 +
                 m_coefs[5]  * mag_1_2 +
                 m_coefs[7]  * mag_1_3 +
                 m_coefs[9]  * mag_1_4 +
                 m_coefs[11] * mag_1_5 +
                 m_coefs[13] * mag_1_6 +
                 m_coefs[15] * mag_1_7
                );

             out_2 = in_2 * (
                 m_coefs[0] +
                 m_coefs[2]  * mag_2_1 +
                 m_coefs[4]  * mag_2_2 +
                 m_coefs[6]  * mag_2_3 +
                 m_coefs[8]  * mag_2_4 +
                 m_coefs[10] * mag_2_5 +
                 m_coefs[12] * mag_2_6 +
                 m_coefs[14] * mag_2_7
                );

             out_3 = in_3 * (
                 m_coefs[1] +
                 m_coefs[3]  * mag_3_1 +
                 m_coefs[5]  * mag_3_2 +
                 m_coefs[7]  * mag_3_3 +
                 m_coefs[9]  * mag_3_4 +
                 m_coefs[11] * mag_3_5 +
                 m_coefs[13] * mag_3_6 +
                 m_coefs[15] * mag_3_7
                );

             out[0] = out_0;
             out[1] = out_1;
             out[2] = out_2;
             out[3] = out_3;
        }
    }

    return dataOut->getLength();
}

void MemlessPoly::set_parameter(const string& parameter, const string& value)
{
    stringstream ss(value);
    ss.exceptions ( stringstream::failbit | stringstream::badbit );

    if (parameter == "ncoefs") {
        throw ParameterError("Parameter 'ncoefs' is read-only");
    }
    else if (parameter == "coeffile") {
        try {
            load_coefficients(value);
            m_coefs_file = value;
        }
        catch (std::runtime_error &e) {
            throw ParameterError(e.what());
        }
    }
    else {
        stringstream ss;
        ss << "Parameter '" << parameter <<
            "' is not exported by controllable " << get_rc_name();
        throw ParameterError(ss.str());
    }
}

const string MemlessPoly::get_parameter(const string& parameter) const
{
    stringstream ss;
    if (parameter == "ncoefs") {
        ss << m_coefs.size();
    }
    else if (parameter == "coefFile") {
        ss << m_coefs_file;
    }
    else {
        ss << "Parameter '" << parameter <<
            "' is not exported by controllable " << get_rc_name();
        throw ParameterError(ss.str());
    }
    return ss.str();
}

