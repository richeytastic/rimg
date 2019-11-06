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

#include <ProHOGExtractor.h>
using rimg::ProHOGExtractor;
using rimg::FeatureExtractor;
#include <sstream>
#include <cassert>


ProHOGExtractor::ProHOGExtractor( int nbins, bool dirDep, cv::Size cellDims, cv::Mat img)
    : FeatureExtractor(img.size()), _nbins(nbins), _dirDep(dirDep), _cellDims(cellDims), _prohog( ProHOG::create( img, nbins, dirDep))
{
    _prohog->setCellDims( cellDims);
}   // end ctor


ProHOGExtractor::ProHOGExtractor()
    : FeatureExtractor(), _nbins(9), _dirDep(true), _cellDims(1,1)
{}   // end ctor



void ProHOGExtractor::getValidImageTypes( vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(BGR);
    vimgTypes.push_back(Grey);
    vimgTypes.push_back(Light);
    vimgTypes.push_back(CIELab);
    vimgTypes.push_back(Depth);
    vimgTypes.push_back(EDT);
}   // end getValidImageTypes



string ProHOGExtractor::getParams() const
{
    std::ostringstream oss;
    oss << _nbins << " " << rlib::asString( _dirDep) << " " << _cellDims.width << " " << _cellDims.height;
    return oss.str();
}   // end getParams


// protected virtual
FeatureExtractor::Ptr ProHOGExtractor::createFromParams( const string& params) const
{
    // No restrictions on image type
    cv::Size cdims;
    int nbins;
    bool dirDep;
    try
    {
        std::istringstream iss(params);
        iss >> nbins;
        dirDep = rlib::asBool( iss);
        iss >> cdims.width;
        iss >> cdims.height;
    }   // end try
    catch ( const std::exception&)
    {
        throw ExtractorTypeException( "Couldn't read ProHOGExtractor params from string: " + params);
    }   // end catch

    ProHOGExtractor *fx = new ProHOGExtractor();
    fx->_nbins = nbins;
    fx->_dirDep = dirDep;
    fx->_cellDims = cdims;
    return FeatureExtractor::Ptr( fx);
}   // end createFromParams



// protected virtual
FeatureExtractor::Ptr ProHOGExtractor::initExtractor( const cv::Mat img) const
{
    //const cv::Mat gcimg = rimg::sqrtGammaCorrect(img);
    return FeatureExtractor::Ptr( new ProHOGExtractor( _nbins, _dirDep, _cellDims, img));
}   // end initExtractor


// protected virtual
cv::Mat_<float> ProHOGExtractor::extractFV( const cv::Rect rct) const
{
    assert( _prohog != NULL);
    return (*_prohog)( rct);
}   // end extractFV
