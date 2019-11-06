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

#pragma once
#ifndef rimg_EDT_FEATURE_EXTRACTOR_H
#define rimg_EDT_FEATURE_EXTRACTOR_H

#include "FeatureExtractor.h"
using rimg::FeatureExtractor;
#include "EDTFeature.h"
using rimg::EDTFeature;

namespace rimg
{

class rimg_EXPORT EDTFeatureExtractor : public FeatureExtractor
{
public:
    EDTFeatureExtractor( int lowCT, int highCT, cv::Size fvDims, cv::Mat img);
    EDTFeatureExtractor();
    virtual ~EDTFeatureExtractor();

    virtual void getValidImageTypes( vector<ImageType>&) const;
    virtual string getTypeString() const { return "EDT";}
    virtual string getParams() const;

    virtual cv::Size getFeatureDims() const { return cv::Size( _fvDims.width*_fvDims.height, 1);}
    virtual cv::Size getMinSamplingDims() const { return cv::Size(2,2);}

    // img: Must be single channel
    // Returns an inverted binary edge map
    static cv::Mat_<byte> createBinaryEdgeMap( const cv::Mat& img, int lowCT, int highCT);

protected:
    virtual FeatureExtractor::Ptr createFromParams( const string&) const;
    virtual FeatureExtractor::Ptr initExtractor( const cv::Mat img) const;
    virtual cv::Mat_<float> extractFV( const cv::Rect rct) const;

private:
    int _lowCT, _highCT;
    cv::Size _fvDims;
    EDTFeature* _edt;
};  // end class

}   // end namespace

#endif
