#ifndef COLORFN_H
#define COLORFN_H

#include <opencv2/opencv.hpp>
#include <vector>

/*
 * This library implied multiple methods to calculate leaf vector.
 * 1. Different color function (diff, ratio, hsv)
 * 2. Different fit method ( wls, ga, sample)
 *
 * Other: diff crop size and diff resolution
 */

namespace lgv {

typedef int (*ColorFn)(int, int, int); // RGB

// Function to find green
int cfn_diff(int, int, int);
int cfn_ratio(int, int, int);
int cfn_hsv(int, int, int);

class Lgvector{
  public:
    float x;
    float y;
    float dx;
    float dy;

    void modratio(float); // Modify x and y by ratio => x *= ratio, y*= ratio
};

typedef std::vector<cv::Point> Points;
typedef std::vector<float> Weights;

// Different Fit Function
Lgvector lmsLineFit( Points );
Lgvector wlsLineFit( Points, Weights );

// Main Function
Lgvector wls_fit(cv::Mat&, ColorFn);
Lgvector ga_fit(cv::Mat&, ColorFn, int = 5, int = 100); // Option: Iterations, Seeds
Lgvector sample_fit(cv::Mat&, ColorFn, int = 100); // Option: Seeds
Lgvector fixsample_fit(cv::Mat&, ColorFn, int = 10); // Option: Sample w and h
void printSlope(Lgvector);
void printSlope(Lgvector, cv::Mat&);

cv::Rect expandBox(cv::Rect,cv::Mat,int);
cv::Rect expandSquBox(cv::Rect,cv::Mat,int);

float error_test(Lgvector&, Lgvector&);

// Dont forgot the crop and resize will increase speed
// And dont forgot the ratio that need to be adjust back

} // namespace

#endif
