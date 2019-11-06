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

#include <HOGExtractor.h>
using rimg::HOGExtractor;
//using rimg::HOG;
#include <sstream>
#include <cassert>


HOGExtractor::HOGExtractor( int nbins, bool dirDep, cv::Size bdims, int cellPxls, cv::Mat img)
    : FeatureExtractor( img.size()), _nbins(nbins), _dirDep(dirDep), _bdims(bdims), _cellPxls(cellPxls), _hog(NULL), _img(img)
{
    if ( bdims.width < 1 || bdims.height < 1)   // HOG should already have checked this potential error
        throw ExtractorTypeException( "Invalid fixed image dimension (multiplier for HOG must be > 0 in width and height)");

    _hog = new cv::HOGDescriptor( cv::Size((bdims.width+1)*cellPxls, (bdims.height+1)*cellPxls),
                                  cv::Size(2*cellPxls, 2*cellPxls), // Block size
                                  cv::Size(cellPxls,cellPxls),      // Block stride
                                  cv::Size(cellPxls,cellPxls),      // Cell size
                                  nbins); // N bins
}   // end ctor


HOGExtractor::HOGExtractor()
    : FeatureExtractor(), _nbins(0), _dirDep(false), _bdims(1,1), _cellPxls(0), _hog(NULL)
{}   // end ctor


HOGExtractor::~HOGExtractor()
{
    if ( _hog != NULL)
        delete _hog;
}   // end dtor


void HOGExtractor::getValidImageTypes( std::vector<ImageType>& vimgTypes) const
{
    vimgTypes.push_back(BGR);
    vimgTypes.push_back(Grey);
    vimgTypes.push_back(Depth);
    vimgTypes.push_back(Light);
    vimgTypes.push_back(EDT);
}   // end getValidImageTypes



string HOGExtractor::getParams() const
{
    std::ostringstream oss;
    oss << _nbins << " " << rlib::asString( _dirDep) << " " << _bdims.width << " " << _bdims.height << " " << _cellPxls;
    return oss.str();
}   // end getParams


// protected
FeatureExtractor::Ptr HOGExtractor::createFromParams( const string& params) const
{
    int nbins, cellPxls;
    bool dirDep;
    cv::Size bdims;
    try
    {
        std::istringstream iss(params);
        iss >> nbins;
        dirDep = rlib::asBool(iss);
        iss >> bdims.width >> bdims.height >> cellPxls;
    }   // end try
    catch ( const std::exception&)
    {
        throw ExtractorTypeException( "Unable to parse params for HOGExtractor: " + params);
    }   // end catch

    if ( bdims.width < 1 || bdims.height < 1)
        throw ExtractorTypeException( "Invalid fixed image dimension (multiplier for HOG must be > 0 in width and height)");

    HOGExtractor* fx = new HOGExtractor;
    fx->_nbins = nbins;
    fx->_dirDep = dirDep;
    fx->_bdims = bdims;
    fx->_cellPxls = cellPxls;
    return FeatureExtractor::Ptr( fx);
}   // end createFromParams


// protected
FeatureExtractor::Ptr HOGExtractor::initExtractor( const cv::Mat img) const
{
    cv::Mat m;
    const int itype = img.type();
    if ( itype == CV_8UC3)
        cv::cvtColor( img, m, cv::COLOR_RGB2GRAY);   /*** OPENCV HOG ***/
        //cv::cvtColor( img, m, CV_RGB2GRAY);   /*** OPENCV HOG ***/
    else if ( itype == CV_32FC1)
    {
#ifndef NDEBUG
        double mn, mx;
        cv::minMaxLoc( img, &mn, &mx);
        assert( mn >= 0 && mx <= 1);
#endif
        img.convertTo( m, CV_8U, 255, 0);
    }   // end if
    else
    {
        m = img;
        assert( itype == CV_8UC1);
    }   // end else

    return FeatureExtractor::Ptr( new HOGExtractor( _nbins, _dirDep, _bdims, _cellPxls, m));
}   // end initExtractor



cv::Mat_<float> HOGExtractor::extractFV( const cv::Rect rct) const
{
    assert( _hog != NULL);

    const cv::Size insz((_bdims.width+1)*_cellPxls, (_bdims.height+1)*_cellPxls);
    cv::Mat mrsz;   // Resize the image extract to the required dimensions
    cv::resize( _img(rct), mrsz, insz);

    std::vector<float> fv;
    _hog->compute( mrsz, fv);
    cv::Mat_<float> fm(1,(int)fv.size());
    memcpy( fm.ptr<float>(), &fv[0], sizeof(float) * fv.size());
    return fm;
}   // end extractFV
