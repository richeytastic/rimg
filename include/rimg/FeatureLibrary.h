/************************************************************************
 * Copyright (C) 2017 Richard Palmer
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

#ifndef rimg_FEATURE_LIBRARY_H
#define rimg_FEATURE_LIBRARY_H

/**
 * Singleton class where all known feature types are registered for use.
 */

#include <string>
using std::string;
#include <istream>
#include <vector>
using std::vector;
#include <boost/unordered_map.hpp>
using boost::unordered_map;
#include <boost/shared_ptr.hpp>

#include "FeatureExceptions.h"
using rimg::ExtractorTypeException;
#include "FeatureUtils.h"

#include "FeatureExtractor.h"
typedef rimg::FeatureExtractor::Ptr FX;

//#include "GallLempitskyFeatureExtractor.h"
#include "DepthDiffExtractor.h"
#include "GradientExtractor.h"
#include "LocalBinaryPatternExtractor.h"
#include "CircleDiffExtractor.h"
#include "FastHOGExtractor.h"
#include "HOGExtractor.h"
#include "ProHOGExtractor.h"
#include "SobelEdgesExtractor.h"
#include "EDTFeatureExtractor.h"


namespace rimg
{

class rimg_EXPORT FeatureLibrary
{
public:
    typedef boost::shared_ptr<FeatureLibrary> Ptr;

    static Ptr get();   // Get the shared feature library

    // Read in feature extractors from a file containing the specifications.
    static std::vector<FX> readFXs( const string& filename);

    void registerFX( FX); // Register a feature extractor that can be built by this factory class

    // Build a single feature extractor
    FX build( const string& fxspec) const throw (ExtractorTypeException);

    // Build several (store in fxs) and return number constructed.
    size_t build( const vector<string>& fxspecs, vector<FX>& fxs) const throw (ExtractorTypeException);

    // Build from input stream
    size_t build( istream&, vector<FX>& fxs) const throw (ExtractorTypeException);

private:
    unordered_map<string, FX> _fxMap;  // Registered FXs

    FeatureLibrary()
    {
        //registerFX( FX( new rimg::GallLempitskyFeatureExtractor));
        registerFX( FX( new rimg::DepthDiffExtractor));
        registerFX( FX( new rimg::GradientExtractor));
        registerFX( FX( new rimg::LocalBinaryPatternExtractor));
        registerFX( FX( new rimg::CircleDiffExtractor));
        registerFX( FX( new rimg::FastHOGExtractor));
        registerFX( FX( new rimg::HOGExtractor));
        registerFX( FX( new rimg::ProHOGExtractor));
        registerFX( FX( new rimg::SobelEdgesExtractor));
        registerFX( FX( new rimg::EDTFeatureExtractor));
    }   // end ctor

    static Ptr S_singleton;
};  // end class

}   // end namespace

#endif




