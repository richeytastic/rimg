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

#include "DepthDiffExtractor.h"
using rimg::PatchPointType;
using rimg::DepthDiffExtractor;
#include <algorithm>
#include <sstream>
#include <cassert>


DepthDiffExtractor::DepthDiffExtractor( PatchPointType ppt, float s, cv::Mat m)
    : FeatureExtractor(m.size()), _ppt(ppt), _sensitivity(s), _depthDiff( new DepthDiff(m, ppt, s))
{}   // end ctor



DepthDiffExtractor::DepthDiffExtractor()
    : FeatureExtractor(), _ppt(FOUR_PT), _sensitivity(1), _depthDiff(NULL)
{}   // end ctor


DepthDiffExtractor::~DepthDiffExtractor()
{
    if ( _depthDiff != NULL)
        delete _depthDiff;
}   // end dtor



void DepthDiffExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(Depth);
    vimgTypes.push_back(EDT);
    vimgTypes.push_back(Light);
}   // end getValidImageTypes



string DepthDiffExtractor::getParams() const
{
    std::ostringstream oss;
    switch (_ppt)
    {
        case FOUR_PT:
            oss << "Four_Pt";
            break;
        case FIVE_PT:
            oss << "Five_Pt";
            break;
        case NINE_PT:
            oss << "Nine_Pt";
            break;
        case THIRTEEN_PT:
            oss << "Thirteen_Pt";
            break;
    }   // end swtich
    return oss.str();
}   // end getParams


PatchPointType fromString( string sppt)
{
    std::transform( sppt.begin(), sppt.end(), sppt.begin(), ::tolower);
    if ( sppt == "four_pt")
        return rimg::FOUR_PT;
    else if ( sppt == "five_pt")
        return rimg::FIVE_PT;
    else if ( sppt == "nine_pt")
        return rimg::NINE_PT;
    else if ( sppt == "thirteen_pt")
        return rimg::THIRTEEN_PT;
    else
        throw ExtractorTypeException("Invalid PatchPointType string!");
    return (PatchPointType)-1;
}   // end fromString



FeatureExtractor::Ptr DepthDiffExtractor::createFromParams( const string& params) const
{
    PatchPointType ppt;
    float sensitivity = 1;
    try
    {
        std::istringstream iss(params);
        string sppt;
        iss >> sppt;
        ppt = fromString(sppt);   // May throw
        iss >> sensitivity;
    }   // end try
    catch ( const std::exception&)
    {
        throw ExtractorTypeException( "Cannot read params for DepthDiffExtractor from string: " + params);
    }   // end catch

    DepthDiffExtractor *dd = new DepthDiffExtractor;
    dd->_ppt = ppt;
    dd->_sensitivity = sensitivity;
    return FeatureExtractor::Ptr(dd);
}   // end createFromParams



FeatureExtractor::Ptr DepthDiffExtractor::initExtractor( const cv::Mat img) const
{
    return FeatureExtractor::Ptr( new DepthDiffExtractor( _ppt, _sensitivity, img));
}   // end initExtractor



/*
size_t DepthDiffExtractor::extract( const cv::Rect rct, PatchDescriptor::Ptr pd) const
{
    assert( _depthDiff != NULL);
    const cv::Mat_<float> fv = (*_depthDiff)(rct);
    assert( fv.rows == 1);
    pd->addFeatureVector(fv);
    return fv.rows;
}   // end extract


size_t DepthDiffExtractor::extract( const cv::Mat m, PatchDescriptor::Ptr pd) const
{
    const cv::Mat_<float> fv = DepthDiff( m, _ppt, _sensitivity)();
    pd->addRowFeatureVectors(fv);
    return fv.rows;
}   // end extract
*/


cv::Mat_<float> DepthDiffExtractor::extractFV( const cv::Rect rct) const
{
    assert( _depthDiff != NULL);
    const cv::Mat_<float> fv = (*_depthDiff)(rct);
    assert( fv.rows == 1);
    return rimg::toRowVector(fv);
}   // end extractFV
