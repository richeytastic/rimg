/************************************************************************
 * Copyright (C) 2019 Richard Palmer
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

#ifndef rimg_H
#define rimg_H

// Disable warnings about standard template library specialisations not being exported in the DLL interface
#ifdef _WIN32
#pragma warning( disable : 4251)
#pragma warning( disable : 4275)
#endif

#include <cmath>
#include <vector>
#include <string>
#include <exception>
#include <stdexcept>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <opencv2/opencv.hpp>

namespace rimg {

using Keypoints = std::vector<cv::KeyPoint>;
using Lines3d = std::vector<cv::Vec6f>;  // 3D lines (x1,y1,z1,x2,y2,z2)
using Lines = std::vector<cv::Vec4i>;
using Circles = std::vector<cv::Vec3f>;

using uint = unsigned int;
using byte = unsigned char;

}   // end namespace

#include "rimg_Export.h"

#include "rimg/AdaptiveDepthPatchScanner.h"
#include "rimg/AdaptiveDepthSegmenter.h"
#include "rimg/AdaptiveDepthStructureFilter.h"
#include "rimg/BatchFeatureExtractor.h"
#include "rimg/CannyOperator.h"
#include "rimg/CircleDiff.h"
#include "rimg/CircleDiffExtractor.h"
#include "rimg/Colour.h"
#include "rimg/ColourDetector.h"
#include "rimg/ConnectedComponents.h"
#include "rimg/ContentFinder.h"
#include "rimg/DescriptorJoiner.h"
#include "rimg/DescriptorStatistics.h"
#include "rimg/DepthDiff.h"
#include "rimg/DepthDiffExtractor.h"
#include "rimg/DepthFinder.h"
#include "rimg/DepthSegmenter.h"
#include "rimg/DistanceTransform.h"
#include "rimg/EDTFeature.h"
#include "rimg/EDTFeatureExtractor.h"
#include "rimg/Equi2Rect.h"
#include "rimg/FastHOG.h"
#include "rimg/FastHOGExtractor.h"
#include "rimg/FeatureExceptions.h"
#include "rimg/FeatureExtractor.h"
#include "rimg/FeatureLibrary.h"
#include "rimg/FeatureOperator.h"
#include "rimg/FeatureUtils.h"
#include "rimg/Fish2Rect.h"
#include "rimg/GrabCutsOperator.h"
#include "rimg/GradientsBuilder.h"
#include "rimg/GradientExtractor.h"
#include "rimg/HaarCascadeDetector.h"
#include "rimg/Histogram.h"
#include "rimg/Histogram1D.h"
#include "rimg/HOGExtractor.h"
#include "rimg/HoughCirclesOperator.h"
#include "rimg/HoughLinesOperator.h"
#include "rimg/ImageHistogram.h"
#include "rimg/ImageLabeller.h"
#include "rimg/ImageProcess.h"
#include "rimg/ImagePyramid.h"
#include "rimg/ImageType.h"
#include "rimg/ImageTypeEnum.h"
#include "rimg/IntensityIndexer.h"
#include "rimg/KeypointsConverter.h"
#include "rimg/KeypointsDetector.h"
#include "rimg/LaplacianZC.h"
#include "rimg/LinesConverter.h"
#include "rimg/LinesFilter.h"
#include "rimg/LocalBinaryPattern.h"
#include "rimg/LocalBinaryPatternExtractor.h"
#include "rimg/NFoldCalculator.h"
#include "rimg/OffsetPatchScanner.h"
#include "rimg/PatchDescriptor.h"
#include "rimg/PatchDescriptorExtractor.h"
#include "rimg/ProHOG.h"
#include "rimg/ProHOGExtractor.h"
#include "rimg/ProHOGTools.h"
#include "rimg/RangePatchScanner.h"
#include "rimg/RectangleManager.h"
#include "rimg/RectCluster.h"
#include "rimg/RegionSorter.h"
#include "rimg/SobelEdges.h"
#include "rimg/SobelEdgesExtractor.h"
#include "rimg/SobelMaker.h"
#include "rimg/VectorDistribution.h"
#include "rimg/WatershedOperator.h"

#endif
