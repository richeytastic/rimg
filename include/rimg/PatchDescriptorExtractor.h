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

#ifndef rimg_PATCH_DESCRIPTOR_EXTRACTOR_H
#define rimg_PATCH_DESCRIPTOR_EXTRACTOR_H

#include <cstdlib>
#include <cassert>
#include <vector>
#include <opencv2/opencv.hpp>
#include "PatchDescriptor.h"
#include "FeatureUtils.h"
#include "FeatureExceptions.h"
#include "AdaptiveDepthPatchScanner.h"  // PatchRanger
#include <rlib/Random.h>
#include "FeatureExtractor.h"
#include "rimg_Export.h"


// Offsets are calculated as proportions of the patch pixel dimensions.
// Therefore, at detection, whether or not fixed pixel or scaled real
// patch dimensions are used, the determination of the actual offset
// distance in pixels will always be <vx*qx, vy*qy> where vx,vy are
// the offset proportions calculated here, and qx,qy are the pixel
// dimensions of the patch calculated for some point in a query image.
namespace rimg {

using FX = rimg::FeatureExtractor::Ptr;

// Extract N patch features and offset vectors from random or known locations over an image of a single object.
// Client is responsible for deleting created PatchDescriptor objects.
class rimg_EXPORT PatchDescriptorExtractor
{
public:
    // For SCALED DIMS patch extraction
    PatchDescriptorExtractor( const cv::Mat_<float>& rng,       // Range for determining patch dims
                              const cv::Mat& mask,              // Mask of locations to take patches from (CV_8UC1)
                              const cv::Rect& boundBox,         // Bounding box of example
                              const cv::Point2f& offsetRef,     // Offset of example reference point (prop of bbox)
                              const cv::Size2f& realPatchSize,  // Real world dimensions of a patch
                              rlib::Random& rndNumGen);         // Random number generator

    // For FIXED DIMS patch extraction
    PatchDescriptorExtractor( const cv::Mat& mask,              // Mask of locations to take patches from (CV_8UC1)
                              const cv::Rect& boundBox,         // Bounding box of example
                              const cv::Point2f& offsetRef,     // Offset of example reference point (prop of bbox)
                              const cv::Size2i& pxlPatchSize,   // Fixed pixel dimensions of a patch
                              rlib::Random& rndNumGen);         // Random number generator

    ~PatchDescriptorExtractor();

    void addFX( const FX& fx);  // Add a pre-processed FX

    // Randomly sample an area of the instance, returning the sampled rectangle from the instance's image.
    // User can keep calling until happy with result, before calling extractPatch to populate the
    // patch descriptor object and return it.
    // If after maxtries, a valid patch couldn't be found (one that's within the image and
    // not smaller than the minimum FX sampling dims), a (0,0,0,0) rectangle is returned.
    const cv::Rect& sampleRandomPatch( int maxtries=50);

    PatchDescriptor::Ptr extractPatch() const;

    // Return the number of pixels within the bounding box example where extracts can
    // be generated randomly from (according to the size of the bounding box and the
    // given mask).
    int getNumSamplingPoints() const { return (int)_samplePts.size();}

    // Supply the image that will be used to show debug output after every call to extractPatch.
    void supplyDebugImage( const cv::Mat_<cv::Vec3b>& img);

private:
    const cv::Mat_<float> _rng;
    const cv::Mat _mask;
    const cv::Rect _bbox;   // Bounding box for instance
    const cv::Size2f _realPatchSize;
    const cv::Size2i _pxlPatchSize;
    const cv::Rect _imgRect;
    const rimg::PatchRanger* _pRanger;

    cv::Rect _prect;
    cv::Point _absPt;

    rlib::Random& _rndNumGen;
    cv::Point _objRefPoint;   // Absolute coordinates of object reference point (nominally the bounding box centre)

    std::vector<FX> _fxs;   // The feature extractors for the patches
    cv::Size _minSamplingDims;

    mutable cv::Mat_<cv::Vec3b> _timg;

    void createBoxSamplePoints();
    std::vector<cv::Point> _samplePts;  // Locations of sample points within the bounding box
};  // end class

}   // end namespace

#endif
