/************************************************************************
 * Copyright (C) 2022 Richard Palmer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ************************************************************************/

#ifndef rimg_COLOUR_H
#define rimg_COLOUR_H

#include "rimg.h"
#include <rlib/Random.h>
#include <cstdlib>

namespace rimg {

class rimg_EXPORT Colour
{
public:
    Colour();
    Colour( const Colour&) = default;
    Colour &operator=( const Colour&) = default;

    Colour( int, int, int);             // Values in [0,255]
    Colour( size_t, size_t, size_t);    // Values in [0,255]

    Colour( double, double, double);    // Values in [0,1]
    explicit Colour( const float*);
    explicit Colour( const double*);

    // Return the RGB components as values in [0,255]
    int ired() const;
    int igreen() const;
    int iblue() const;

    const double& operator[]( int) const;
    bool operator==( const Colour&) const;

    static Colour hsv2rgb( const Colour&);
    static Colour rgb2hsv( const Colour&);

    static Colour white();
    static Colour black();
    static Colour red();
    static Colour green();
    static Colour blue();

    // Return a random colour with the constraints that it can be no
    // darker or lighter across the colours than the given colours.
    static Colour random( const Colour &min=black(), const Colour &max=white());

private:
    double _vals[3];
    double& operator[]( int);
    void _set( double, double, double);
    static rlib::Random s_rng;
};  // end class

struct rimg_EXPORT HashColour { size_t operator()( const Colour&) const;};

}   // end namespace

#endif

