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

#ifndef rimg_FEATURE_UTILS_H
#define rimg_FEATURE_UTILS_H

#ifdef _WIN32
// Disable warnings about MSVC compiler not implementing exception specifications
#pragma warning( disable : 4290)
// Disable warnings about standard template library specialisations not being exported in the DLL interface
#pragma warning( disable : 4251)
#pragma warning( disable : 4275)
#endif

#include "rimg_Export.h"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <ostream>
#include <istream>
#include <opencv2/opencv.hpp>
#include "FeatureExceptions.h"

namespace rlib{ class Random;}

namespace rimg {

using byte = unsigned char;

using std::ostream;
using std::istream;
using std::vector;
using std::string;

// Convenience class for single channel images whose depth we don't care about.
// Provided image must be continuous.
class rimg_EXPORT DepthAgnosticImg
{
public:
    DepthAgnosticImg( const cv::Mat& img);   // img must be single channel, but any depth
    double operator()( const cv::Point& p) const;   // Get the value at p

private:
    const cv::Mat _img;
    const byte* _imgArr;
    const int _rows, _cols, _depth, _elemSz;
    const cv::Rect _imgRct;
};  // end class


// Transform and return the given vertex by the given matrix (T*v).
rimg_EXPORT cv::Vec3f transform( const cv::Matx44d& T, const cv::Vec3f& v);
rimg_EXPORT cv::Vec3d transform( const cv::Matx44d& T, const cv::Vec3d& v);

// Transform v by T in place.
rimg_EXPORT void transform( const cv::Matx44d& T, cv::Vec3f& v);
rimg_EXPORT void transform( const cv::Matx44d& T, cv::Vec3d& v);


// Calculate and return barycentric coordinates of p in triangle ABC. Returns u,v,w where w = 1-u-v and u+v+w=1.
// p = u*A + v*B + w*C.
template <typename T>
cv::Vec3d calcBarycentric( const cv::Vec<T,3> &A, const cv::Vec<T,3> &B, const cv::Vec<T,3> &C, const cv::Vec<T,3> &p);

// More general form uses Heron's forumla for calculating triangle areas.
template <typename T, int c>
cv::Vec3d calcBarycentric( const cv::Vec<T,c> &A, const cv::Vec<T,c> &B, const cv::Vec<T,c> &C, const cv::Vec<T,c> &p);

// Calculate triangle area from vertices ABC using cross product (magnitude of cross product is area of parallelogram).
template <typename T>
double calcTriangleArea( const cv::Vec<T,3> &A, const cv::Vec<T,3> &B, const cv::Vec<T,3> &C);

// For dimensions other than 3, lengths of triangle sides are taken and Heron's forumula is used.
template <typename T, int c>
double calcTriangleArea( const cv::Vec<T,c> &A, const cv::Vec<T,c> &B, const cv::Vec<T,c> &C);

// Calculate the area of a triangle given 3 side lengths (by Heron's formula).
rimg_EXPORT double calcTriangleArea( double a, double b, double c);
rimg_EXPORT float calcTriangleArea( float a, float b, float c);


// On return, v0 <= v1 <= v2 will be true.
template <typename T>
void reorderAscending( T& v0, T& v1, T& v2);

// Given integral image X, calculate and return the sum over the given rectangle.
template <typename T>
T getIntegralImageSum( const cv::Mat& iimgX, const cv::Rect& rct);

// Return the minimum of values a,b,c.
template <typename T>
T min( T a, T b, T c);

// Return the minimum of values a,b.
template <typename T>
T min( T a, T b);

// Return the maximum of values a,b,c.
template <typename T>
T max( T a, T b, T c);

// Return the maximum of values a,b.
template <typename T>
T max( T a, T b);

// Given integral image X and the integral image of its square values
// (calculate using cv::integral(InputArray src, OutputArray sum, OutputArray sqsum))
// calculate the uncorrected variance over the given rectangle. Take sqrt for standard deviation.
// maskii (if given) is the integral image of counts to be used in the calculation which MUST
// have been used previously to create iimgX and iimgXsq! (it cannot be passed arbitrary and a
// correct result be returned).
// maskii should be created using a function such as createMaskIntegralImage (below).
// T is the template parameter of iimgX.
template <typename T>
double calcVariance( const cv::Mat& iimgX, const cv::Mat_<double>& iimgXsq, const cv::Rect& rct, cv::Mat_<int> maskii=cv::Mat_<int>());

template <typename T>
cv::Mat_<int> createMaskIntegralImage( const cv::Mat& X, T minX, T maxX);

// Given a rectangle within bounding dimensions sz0, find and return the proportionately positioned
// and dimensioned rectangle that fits within the dimensions sz1.
rimg_EXPORT cv::Rect calcRelativeRect( const cv::Rect& rct0, const cv::Size& sz0, const cv::Size& sz1);

rimg_EXPORT cv::Rect createRandomRect( const cv::Size& bounds, const cv::Size& minSz, rlib::Random* rndGen);

// Scale the rectangle by the given factor (returns as well for convenience).
rimg_EXPORT cv::Rect& scale( cv::Rect& rect, float factor);

// Get the point that is offset from the top left corner of rct by offset as a proportion of the width
// and height of rct. So, for example, to return the point at the centre of rct, offset should be 0.5, 0.5.
// For the point at the centre bottom, offset should be 0.5, 1.0. And so on. Obviously, offset points outside
// the boundary of rct can be calculated.
rimg_EXPORT cv::Point calcPixelOffset( const cv::Rect& rct, const cv::Point2f& offset);

// As above, but returns a floating point value.
rimg_EXPORT cv::Point2f calcOffset( const cv::Rect_<double>& rct, const cv::Point2f& offset);

// Calculate the offset into the rotated rectangle from the top left corner of the rectangle
// assuming a coordinate system relative to the angle of the rectangle.
rimg_EXPORT cv::Point2f calcOffset( const cv::RotatedRect&, const cv::Point2f& offset);

// Get the point in the middle of the given rectangle (rounded) (special case of calcPixelOffset above).
rimg_EXPORT cv::Point calcPixelCentre( const cv::Rect& rct);

// Convenience functions to check if cv::Rect inner is within
// the bounds of the given outer cv::Rect, or the rows and cols of m.
rimg_EXPORT bool isWithin( const cv::Rect& outer, const cv::Rect& inner);
rimg_EXPORT bool isWithin( const cv::Mat& m, const cv::Rect& inner);
rimg_EXPORT bool isWithin( const cv::Mat& m, const cv::Point& inner);
rimg_EXPORT bool isContained( const cv::Rect& outer, const cv::Rect& inner);   // Synonymous with isWithin
rimg_EXPORT bool isContained( const cv::Mat& m, const cv::Rect& inner);    // Synonymous with isWithin
rimg_EXPORT bool isContained( const cv::Mat& m, const cv::Point& inner);    // Synonymous with isWithin

// Given a single channel image of any depth, find the point giving a
// local minimum closest to startPoint between p0 and p1.
rimg_EXPORT cv::Point findLocalMin( const cv::Mat& singleChannelImage,
                        const cv::Point& startPoint, const cv::Point& p0, const cv::Point& p1);

// Find the global minimum value in the given image between points p0 and p1.
// Returns the point and sets out param minVal to the value if not null.
rimg_EXPORT cv::Point findMin( const cv::Mat_<float>&, const cv::Point& p0, const cv::Point& p1, float* minVal=nullptr);

// Find the sum of values between points p0 and p1 where only values > x and < y are counted.
// On return, out parameter count will be the number of values between p0 and p1 that are strictly > x and < y.
// Type parameter must be the scalar type of the matrix.
template <typename T>
T findSumBetweenPoints( const cv::Mat& mat, const cv::Point2f& p0, const cv::Point2f& p1, T x, T y, int& count);

// Treats the provided image as a depth map and finds the maximum difference in depth
// as measured orthogonally to the base vector between points p1 and p0. Returns the depth
// delta itself, and copies the location of this point to dpoint on exit.
rimg_EXPORT float findOrthogonalMaxDelta( const cv::Mat_<float>&, const cv::Point& p0, const cv::Point& p1, cv::Point& dpoint);

// Find the mean position of the non-zero pixels of mask.
// Returns true if mask contains at least one non-zero pixel.
rimg_EXPORT bool findMeanPosition( cv::Point& p, const cv::Mat_<byte> mask);

// Find and return the first point from p1 to p2 (exclusive) that intersects with a non-zero pixel of m.
// If no non-zero pixel is found on the way to p2, cv::Point(-1,-1) is returned.
rimg_EXPORT cv::Point findIntersectionPoint( const cv::Mat_<byte>& m, const cv::Point& p1, const cv::Point& p2);

// As above, but once a boundary pixel is hit, the function only returns once a zero pixel is hit.
rimg_EXPORT cv::Point findOuterIntersectionPoint( const cv::Mat_<byte>& m, const cv::Point& p1, const cv::Point& p2);

// All non-zero element indices of img are extracted as Points and placed in provided vector.
// Useful for functions like cv::convexHull which takes a vector of points as input.
// Returns the number of points appended to pts.
rimg_EXPORT int nonZeroToPoints( const cv::Mat_<byte>& img, std::vector<cv::Point>& pts);

// Set boundingBox to be the rectangle of minimum area that completely encloses the
// non-zero points of img. Returns the number of points in img that are non-zero.
rimg_EXPORT int findBoundingBox( const cv::Mat_<byte>& img, cv::Rect& boundingBox);

// Creates a mask where each point in provided vector pts is set to 255 in the returned
// image (all other pixels being set to zero).
rimg_EXPORT cv::Mat_<byte> pointsToMask( const cv::Size& imgSz, const std::vector<cv::Point>& pts);

// Zeros out those areas of inputImg where the corresponding value of mask == 0.
// Images must be the same size, but inputImg can be any depth or number of channels.
rimg_EXPORT void setMasked( cv::Mat inputImg, const cv::Mat_<byte> mask);

// Draws the convex hull of the given vector of points as a filled (by default) polygon on the given image.
// img must have dimensions large enough to contain the points given in the vector.
rimg_EXPORT void drawConvexHull( const vector<cv::Point>& points, cv::Mat img, cv::Scalar colour, bool filled=true);

rimg_EXPORT void drawFilledPoly( const vector<cv::Point>& points, cv::Mat img, cv::Scalar colour);

rimg_EXPORT void drawPoly( const vector<cv::Point>& points, cv::Mat img, cv::Scalar colour);

// Create a histogram of the values of the byte image m. The given vector is cleared and resized to 256 before use.
rimg_EXPORT void createHistogram( const cv::Mat_<byte>& m, vector<int>& hist);

// Given a histogram with two expected peaks, find the normal distributions (means and std-devs) that
// best match these peaks by performing k-means clustering (where k=2) on the given histogram.
// This function obviously expects that the peaks can be roughly approximated by normal
// distributions if the output is to make sense!
// The function returns the value between the two Gaussian means where they intersect.
// If the function returns < 0, there is no intersection of the Gaussians between their
// means, implying that the given histogram is not bimodal!
//double calcBimodalGaussians( const vector<int>& hist, double& u1, double& s1, double& u2, double& s2);

rimg_EXPORT cv::Mat_<cv::Vec3b> drawHistogram( const vector<int>& hist, const cv::Size& imgSz, bool makeProbDist=false);

// Creates and returns a normalised 2D Gaussian of given size.
// xcentre, ycentre: give centre coordinates of the 2D Gaussian as proportion of the given size.
rimg_EXPORT cv::Mat_<float> make2DGaussian( const cv::Size filterSz, double xcentre=0.5, double ycentre=0.5);

// Take a N channelled R*C sized input matrix of any depth and turn into a single
// channelled matrix with N rows with each row R*C in length.
// Optional scaling is provided with params alpha and beta (see OpenCV docs on Mat::convertTo).
rimg_EXPORT cv::Mat_<float> toRowVectors( const cv::Mat& img, double alpha=1, double beta=0);

// Goes one further than toRowVectors() and turns an N channelled R*C sized input matrix into a single row vector N*R*C in length.
rimg_EXPORT cv::Mat_<float> toRowVector( const cv::Mat& img, double alpha=1, double beta=0);

// Set the given pixel (not accounting for image channels) to be scaled to its sqrt
// multiplied by sqrt(MAX_VALUE).
rimg_EXPORT void sqrtGammaCorrect( cv::Mat &img, const int MAX_VALUE, int row, int col);

// Gamma correct the given CV_8U image, returning a new image.
rimg_EXPORT cv::Mat sqrtGammaCorrect( const cv::Mat& img);

// Gets the bands of the CIE Lab space for the given image.
// Lightness (channel 0) values are in range [0,100].
// Colour a (channel 1) values are in the range [-127,127].
// Colour b (channel 2) values are in the range [-127,127].
rimg_EXPORT vector<cv::Mat_<float> > getCIELabChannels( const cv::Mat_<cv::Vec3b>& image);

// In-place conversion to CIE-Lab. All values in all channels in range [0,255].
rimg_EXPORT void convertToCIELab( cv::Mat_<cv::Vec3b>& img);

// Gets the CIE Lab lightness channel only with values scaled from 0 to scale.
// Default returned image type is CV_32F
rimg_EXPORT cv::Mat getLightness( const cv::Mat_<cv::Vec3b>& image, double scale=1, int imgType=CV_32F);

// Uses DepthSegmenter to create a depth mask from the most common depth values in dimg.
rimg_EXPORT cv::Mat_<byte> createDepthMask( const cv::Mat_<float> dimg);

// Type-casted retrieval of a value from a pointer into a matrix.
// Takes the following matrix depths:
// CV_8U, CV_8S, CV_16U, CV_16S, CV_32S, CV_32F, CV_64F
rimg_EXPORT double pval( int matrixDepth, const byte* valuePointer);

// Get the horizontal gradient at the given pixel. Boundary pixels take as the gradient
// the difference between the specified pixel and the adjacent inward pixel.
// This function only accepts images having a vector type (cv::Vec) as their element e.g.
// to call for a CV_64FC1 image, the function should be called as
// calcHorizontalGrad<cv::Vec<double,1> >( ...)
template <typename T>
double calcHorizontalGrad( const cv::Mat_<T> &image, int row, int col, int channel);

rimg_EXPORT double calcHorizontalGrad( const cv::Mat& img, int row, int col, int channel);

template <typename T>
double calcHorizontalGrad2( const cv::Mat_<T> &image, int row, int col, int channel);

// Get the vertical gradient at the given pixel. Boundary pixels take as the gradient
// the difference between the specified pixel and the adjacent inward pixel.
// See calcHorizontalGrad for details on required type (must be cv::Vec).
template <typename T>
double calcVerticalGrad( const cv::Mat_<T> &image, int row, int col, int channel);

rimg_EXPORT double calcVerticalGrad( const cv::Mat& img, int row, int col, int channel);

template <typename T>
double calcVerticalGrad2( const cv::Mat_<T> &image, int row, int col, int channel);

// Display a cv::Rect to an outstream.
template <typename T>
ostream &operator<<( ostream&, const cv::Rect_<T>&);

template <typename T>
istream &operator>>( istream&, cv::Rect_<T>&);

rimg_EXPORT ostream &operator<<( ostream&, const cv::Rect&);
rimg_EXPORT istream &operator>>( istream&, cv::Rect&);

// Display a cv::Size to an outstream.
template <typename T>
ostream &operator<<( ostream&, const cv::Size_<T>&);

template <typename T>
istream &operator>>( istream&, cv::Size_<T>&);

rimg_EXPORT ostream &operator<<( ostream&, const cv::Size&);
rimg_EXPORT istream &operator>>( istream&, cv::Size&);

// Writes the given descriptor out the provided stream as single precision
// values separated by spaces on a single line. Does not write newline character.
// Used to write multiple descriptors (one per line) to file.
// Returns the number of values written == descriptor.total()
rimg_EXPORT int writeDescriptor( ostream& os, const cv::Mat& descriptor);

// Read descriptors (written using writeDescriptor above) into a returned matrix.
// Descriptors will be set as either column vectors (asCols=true : the default)
// with the number of rows being equal to the length of the feature vectors and
// the number of columns being equal to the number of descriptors read in,
// or as row vectors with the number of columns being equal to the length of
// the feature vectors and the number of rows equalling the number of descriptors read in.
rimg_EXPORT cv::Mat_<float> readDescriptors( const string fname, bool asCols=true) throw (DescriptorLengthException);

// Read in a single descriptor as a column or row vector.
rimg_EXPORT cv::Mat_<float> readDescriptor( istream& is, bool asCol=true);

// Combine f1 and f2 of different number of columns(rows) but same number of rows(columns)
// into a single row vector of length f1.total() + f2.total()
rimg_EXPORT cv::Mat_<float> combine( const cv::Mat_<float>& f1, const cv::Mat_<float>& f2);

// Write a cv::Mat out to the provided stream in binary format.
// Type of matrix does NOT need to be known!
rimg_EXPORT ostream &writeBinary( ostream &os, const cv::Mat &m);

rimg_EXPORT bool saveBinaryImage( const string fname, const cv::Mat& m);

// Read in a matrix from a stream of binary data. Type does NOT have
// to be known before calling!
rimg_EXPORT istream &readBinary( istream &is, cv::Mat &m);

rimg_EXPORT bool loadBinaryImage( const string fname, cv::Mat& m);

// Returns depth of image as a string e.g. CV_64F
rimg_EXPORT string imgDepthToString( const cv::Mat&);
rimg_EXPORT string imgDepthToString( int depth);

// Returns type of image as string (i.e. depth and number of channels) e.g. CV_8UC3
rimg_EXPORT string imgTypeToString( const cv::Mat&);

// Load an image from the provided absolute filename into the provided cv::Mat header.
// Set bw to true if image should be loaded in single channel grey scale.
rimg_EXPORT bool loadImage( const string &fname, cv::Mat &img, bool bw=false);

// Save the given image to provided filename.
rimg_EXPORT bool saveImage( const string &fname, const cv::Mat &img);

// Shows the provided image in its own main window. Set wait to true to
// cause this function to block until user presses a key.
rimg_EXPORT void showImage( const cv::Mat &img, const string &title, bool wait=false);

// Close the window displaying the image with the given title.
rimg_EXPORT void closeImage( const string& title);

// Display provided image dimensions and number of channels to given output stream.
rimg_EXPORT ostream &print( ostream &os, const cv::Mat &img);

// Takes a single or triple channel image and converts it to a CV_8U image. If any channels
// have values outside the displayable range of [0,255], they are first scaled to be within
// this range. The maximum variance out of all channels is used to scale the values over all
// channels (for triple channel images) so channel value scalings are proportionate.
// If forceScale is true, the image will have its values contrast scaled within the
// displayable range of [0,255] even if its values are currently within this range
// (this will result in a contrast stretching of the image values).
rimg_EXPORT cv::Mat convertForDisplay( const cv::Mat &img, bool forceScale=false);

// Convert a single channel scalar image (e.g. of type float, double, int etc)
// to a triple channel byte grey scale image ready for display (CV_8UC3).
// Parameter invert does value inversion before mapping to the returned matrix.
rimg_EXPORT cv::Mat_<cv::Vec3b> convertFromSingleChannel( const cv::Mat&, bool invert=false);

// Contrast stretch and translate the values of the provided image to lie
// within the provided range [minVal, maxVal].
rimg_EXPORT cv::Mat rescale( const cv::Mat img, double minVal=0, double maxVal=255);

// Converts single channel m (any depth) to contrast stretched byte image with
// values in the range 0 to 255. Values to stretch are taken from the given mask.
// If the mask is left empty, the min and max values from the whole image are
// used to contrast stretch.
rimg_EXPORT cv::Mat_<byte> contrastStretch( const cv::Mat& m, const cv::Mat_<byte> mask=cv::Mat());

rimg_EXPORT cv::Mat_<float> truncateAndScale( const cv::Mat_<float> img, float trunc, float scale=1.0);

// Straight forward function for converting a range map to a contrast scaled CV_8UC3
// for some OpenCV functions.
rimg_EXPORT cv::Mat_<cv::Vec3b> makeCV_8UC3( const cv::Mat_<float> rmap);

// Scale image to given size and return (simply wraps a returning version of cv::resize).
rimg_EXPORT cv::Mat resize( const cv::Mat img, const cv::Size& newSz);

// Shrink and return a copy of img to be no larger than maxd rows/cols.
rimg_EXPORT cv::Mat shrinkMax( const cv::Mat img, size_t maxd);

// Horizontally concatenate a bunch of cv::Mat (which must all be of the same type)
// into a single returned image where the height of the returned image is the maximum
// height (#rows) from all of the input images. Any input image having a smaller height
// is resized up. If not null, cols will be set with the column index that each
// concatentated (and possibily resized) image starts at in the returned image (cols
// is resized to be imgs.size()). This is useful so that position references to the
// input images can be recalculated to account for the new image sizes and positions.
// If the input images are not of the same type, an empty cv::Mat is returned.
rimg_EXPORT cv::Mat concatHorizontalMax( const std::vector<cv::Mat>& imgs, std::vector<int>* cols=nullptr);

// Flattens all N channels of the provided image into a single channel
// by summing each channel/N.
rimg_EXPORT cv::Mat flatten( const cv::Mat &img);

// Do cross-convolution of krn with img and return a response map.
rimg_EXPORT cv::Mat convolve( const cv::Mat &img, const cv::Mat &krn);

// Shows each image plane of an image one by one.
// Each image is contrast scaled within the range [0,255].
rimg_EXPORT void showScaledPlanes( const cv::Mat &img, const string &winNamePrefix);

// Draw the provided rectangles to the provided image with provided
// line thickness (in pixels) and colour (BGR).
rimg_EXPORT void drawBoxes( cv::Mat &img, const vector<cv::Rect> &boxes, int thick=1,
                                const cv::Scalar col=cv::Scalar(255,255,255));

// Draws the rotated rectangle to the image, clipping the lines as needed so that the
// rectangle is not drawn outside of the image canvas.
rimg_EXPORT void drawRotatedRect( cv::Mat& img, const cv::RotatedRect&, int thick=1,
                                const cv::Scalar col=cv::Scalar(255,255,255));

// Find the x's where two normal distributions (with given std devs and means) cross.
rimg_EXPORT void calcNormalsCrossings( double s1, double s2, double u1, double u2, double &x1, double &x2);

// Calculate the value of the normal distribution (with sigma s and mean u) at x.
rimg_EXPORT double calcNormal( double s, double u, double x);

// Round v to the closest multiple of m
rimg_EXPORT int roundMult( double v, int m);

// Calc and return the projection of p along base.
rimg_EXPORT cv::Vec3f project( const cv::Vec3f& p, const cv::Vec3f& base);

// Calculate the sum of the square differences of the provided list of values with the given mean.
// Divide through the returned value with the number of samples (or the number of samples -1) to
// get the variance. calcStdDev (below) uses this function.
rimg_EXPORT double calcSumSqDiffs( const vector<double>& vals, double mean);

// Calculate and return std deviation (using n-bias to account for sample bias)
rimg_EXPORT double calcStdDev( const vector<double> &vals, double mean, int bias=1);

// Flip every instance of dt about the vertical axis
rimg_EXPORT void vertFlipReplace( vector<cv::Mat> &v);

// Create a random colour (BGR) object with given minimum channel intensities
// (set in provided in/out cv::Scalar param).
rimg_EXPORT void createRandomColour( cv::Scalar&, rlib::Random* rnd);

// Change whatever fv is into the provided type as a single channel, single row vector
// so that for returned image m, m.cols = m.total(). Parameter alpha allows for element
// scaling on conversion.
rimg_EXPORT cv::Mat modifyDescriptor( const cv::Mat &fv, int type=CV_32F, float alpha=1);

// Apply yaw, pitch and roll (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rimg_EXPORT cv::Vec3d applyYawPitchRoll( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

// Apply roll, pitch and yaw (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rimg_EXPORT cv::Vec3d applyRollPitchYaw( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

// Apply pitch, roll and yaw (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rimg_EXPORT cv::Vec3d applyPitchRollYaw( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

// Apply roll, yaw and pitch (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rimg_EXPORT cv::Vec3d applyRollYawPitch( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

// Apply pitch, yaw and roll (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rimg_EXPORT cv::Vec3d applyPitchYawRoll( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

// Apply yaw, roll and pitch (in that order) to the given parameter vector and return the result.
// All angles should be given in degrees. Yaw is about Z, pitch is about X, roll is about Y.
rimg_EXPORT cv::Vec3d applyYawRollPitch( const cv::Vec3d& initVec, double yaw, double pitch, double roll);

// Return the squared L2-norm.
template <typename T>
T l2sq( const cv::Vec<T,2>&);

// Return the squared L2-norm.
template <typename T>
T l2sq( const cv::Vec<T,3>&);

// Convert the given image into a displayable range map with values closer to the image plane
// being brighter and dropping to black as distance increases. Range values outside of the given
// min and max ranges are ignored. If maxRng is left as -1, the full range of values in the range
// image will be used to create the grey scale values. Note that different images will have different
// maximum range values so the apparent depth scaling will not be comparable. For comparable apparent
// depth scaling across images, a set min and max range value should be used.
rimg_EXPORT cv::Mat_<byte> makeDisplayableRangeMap( const cv::Mat_<float>& rngImg,
                                                         float minRng=0, float maxRng=-1);

// Create maps of (absolute) change in both the horizontal (hmap) and vertical (vmap) directions.
// If mask is given, only calculate those values where mask != 0.
// img must be single channel of any depth.
rimg_EXPORT void createChangeMaps( const cv::Mat& img, cv::Mat& hmap, cv::Mat& vmap,
                                        bool useAbsolute=false, cv::Mat_<byte> mask=cv::Mat_<byte>());

#include "FeatureUtils.cpp"

}   // end namespace

#endif
