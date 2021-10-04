/*
 * lgv.cpp
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */

#include "lgv.h"
#include <cmath>
#include <cassert>
#include <random>
namespace lgv {

void Lgvector::modratio(float r) {
    this->x *= r;
    this->y *= r;
}

int cfn_diff(int r, int g, int b) {
    int base = g-r-b; // -510 ~ 255
    int relu = base > 0 ? base : 0; // 0 ~ 255
    int stretch = relu*relu/255; // 0 ~ 255
    return stretch;
}

int cfn_ratio(int r, int g, int b) {
    int sum = r+g+b; // 0 ~ 765
    if ( sum == 0 ) {
        return 0;
    }
    int base = 255*g/sum;
    int relu = base > 0 ? base : 0; // 0 ~ 255
    int stretch = relu*relu/255; // 0 ~ 255
    return stretch;
}


int cfn_hsv(int r, int g, int b) {

    // Convert to hsv
    double _r = r / 255.0;
    double _g = g / 255.0;
    double _b = b / 255.0;

    // h, s, v = hue, saturation, value
    double maxrg = _r > _g ? _r : _g;
    double minrg = _r < _g ? _r : _g;
    double cmax = maxrg > _b ? maxrg : _b;
    double cmin = minrg < _b ? minrg : _b;
    double diff = cmax - cmin; // diff of cmax and cmin.
    double h = -1;

    // if cmax and cmax are equal then h = 0
    if (cmax == cmin) {
        h = 0;
    } else if (cmax == _r) {
        h = remainder((60 * ((_g - _b) / diff) + 360) ,360);
    } else if (cmax == _g) {
        h = remainder((60 * ((_b - _r) / diff) + 120) ,360);
    } else if (cmax == _b) {
        h = remainder((60 * ((_r - _g) / diff) + 240) ,360);
    }
    int base = h+201;
    int relu = base > 0 ? base : 0; // 0 ~ 255
    int stretch = relu*relu/255; // 0 ~ 255

    return stretch;

}

Lgvector lmsLineFit( Points points ) {
    double x = 0, y = 0, x2 = 0, y2 = 0, xy = 0, w = 0;
    double dx2, dy2, dxy;
    float t;

    for( int i = 0; i < points.size(); i += 1 ) {
        x +=  points[i].x;
        y +=  points[i].y;
        x2 += points[i].x * points[i].x;
        y2 += points[i].y * points[i].y;
        xy += points[i].x * points[i].y;
    }
    w = points.size();

    x /= w;
    y /= w;
    x2 /= w;
    y2 /= w;
    xy /= w;

    dx2 = x2 - x * x;
    dy2 = y2 - y * y;
    dxy = xy - x * y;

    t = (float) atan2( 2 * dxy, dx2 - dy2 ) / 2;
    return Lgvector{ (float) x, (float) y, (float) cos( t ), (float) sin( t ) };
}
Lgvector wlsLineFit( Points points, Weights weights) {
    assert(points.size() == weights.size());

    double x = 0, y = 0, x2 = 0, y2 = 0, xy = 0, w = 0;
    double dx2, dy2, dxy;
    float t;

    for( int i = 0; i < points.size(); i += 1 ) {
        x += weights[i] * points[i].x;
        y += weights[i] * points[i].y;
        x2 += weights[i] * points[i].x * points[i].x;
        y2 += weights[i] * points[i].y * points[i].y;
        xy += weights[i] * points[i].x * points[i].y;
        w += weights[i];
    }

    x /= w;
    y /= w;
    x2 /= w;
    y2 /= w;
    xy /= w;

    dx2 = x2 - x * x;
    dy2 = y2 - y * y;
    dxy = xy - x * y;

    t = (float) atan2( 2 * dxy, dx2 - dy2 ) / 2;

    return Lgvector{ (float) x, (float) y, (float) cos( t ), (float) sin( t ) };
}
Lgvector ga_fit( cv::Mat& img, ColorFn cfn, int iter_times, int seed_num) {
  int pw = img.size().width;
  int ph = img.size().height;

  /* Get random point */
  /* --------------- */
  // random machine
  std::random_device rd;
  std::mt19937 gen(rd());

  // random get pixels
  std::uniform_int_distribution<> dis_h(0, ph-1);
  std::uniform_int_distribution<> dis_w(0, pw-1);

  Points seeds; 

  // init
  for (int i=0; i<seed_num; i++)
    seeds.push_back(cv::Point(dis_w(gen), dis_h(gen)));

  // iter times
  for (int it=0; it<iter_times; it++) {
    // add more point
    for (int i=0; i<seed_num; i++)
      seeds.push_back(cv::Point(dis_w(gen), dis_h(gen)));
    // get weights
    Weights weights;
    /* int s_total = 0; // cal avg */

    for (auto& s:seeds){
      auto p = img.at<cv::Vec3b>(s.y, s.x);
      // input RGB pixel, return weight. Note that return BGR
      int score = cfn(p.val[2], p.val[1], p.val[0]);
      weights.push_back(score);

      /* s_total+=score; */
    }

    // sample by weight
    Points resamples;
    for (int i=0; i<seed_num; i++) {

      // establish distribution machine
      std::discrete_distribution<> d(weights.begin(), weights.end());

      // get one sample
      int seed_num = d(gen);

      resamples.push_back(seeds[seed_num]);
      seeds.erase(seeds.begin()+seed_num);
      weights.erase(weights.begin()+seed_num);
    }
    seeds.clear();
    seeds = resamples;
  }
  return lmsLineFit( seeds );
}

Lgvector sample_fit(cv::Mat& img, ColorFn cfn, int seed_num) {
  int pw = img.size().width;
  int ph = img.size().height;

  /* Get random point */
  /* --------------- */
  // random machine
  std::random_device rd;
  std::mt19937 gen(rd());

  // random get pixels
  std::uniform_int_distribution<> dis_h(0, ph-1);
  std::uniform_int_distribution<> dis_w(0, pw-1);

  Points seeds; 
  Weights weights;

  // init
  for (int i=0; i<seed_num; i++)
    seeds.push_back(cv::Point(dis_w(gen), dis_h(gen)));

  for (auto& s:seeds){
    auto p = img.at<cv::Vec3b>(s.y, s.x);
    int score = cfn(p.val[2], p.val[1], p.val[0]);
    weights.push_back(score);
  }
  return wlsLineFit( seeds, weights);

}

Lgvector fixsample_fit(cv::Mat& img, ColorFn cfn, int whratio) {
  int pw = img.size().width;
  int ph = img.size().height;
  int w_pitch = pw/whratio;
  int h_pitch = ph/whratio;

  Points points; 
  Weights weights;

  // init
  for (int i=0; i<whratio; i++) {
    for (int j=0; j<whratio; j++) {
        points.push_back(cv::Point(i*w_pitch, j*h_pitch));
    }
  }

  for (auto& s:points){
    auto p = img.at<cv::Vec3b>(s.y, s.x);
    int score = cfn(p.val[2], p.val[1], p.val[0]);
    weights.push_back(score);
  }
  return wlsLineFit( points, weights);

}

Lgvector wls_fit(cv::Mat& img, ColorFn cfn) {
    Points points;
    Weights weights;
    for(int r = 0; r < img.rows; r++) {
        cv::Vec3b* ptr = img.ptr<cv::Vec3b>(r);
        for(int c = 0; c < img.cols; c++) {
            float temp = cfn(ptr[c][2], ptr[c][1], ptr[c][0]);
            temp = temp > 255 ? 255 : temp;
            points.push_back(cv::Point(c, r));
            weights.push_back(temp);
        }
    }
    return wlsLineFit( points, weights);
}

void printSlope(Lgvector v) {
  std::cout << "(" << "vx: " << v.dx << "|" << "vy: " << v.dy << "|" << "x0: " << v.x << "|" << "y0: " << v.y << ")" << std::endl;
}

void printSlope(Lgvector v, cv::Mat& img) {
  int pw = img.size().width;
  int lefty = -v.x*v.dy/v.dx + v.y;
  int righty = (pw-v.x)*v.dy/v.dx + v.y;

  cv::line(img,
      cv::Point(pw-1,righty),
      cv::Point(0,lefty),
      cv::Scalar(0,0,200), 3, 4);
  cv::circle(img, cv::Point(v.x,v.y), 3,cv::Scalar(255,0,0),-1);
}

cv::Rect expandBox(cv::Rect boundingBox,cv::Mat frm,int padding) {
  cv::Rect returnRect = cv::Rect(
          boundingBox.x - padding,
          boundingBox.y - padding,
          boundingBox.width + (padding * 2),
          boundingBox.height+ (padding * 2));
  if (returnRect.x < 0)returnRect.x = 0;
  if (returnRect.y < 0)returnRect.y = 0;
  if (returnRect.x+returnRect.width >= frm.cols)returnRect.width = frm.cols-returnRect.x;
  if (returnRect.y+returnRect.height >= frm.rows)returnRect.height = frm.rows-returnRect.y;
  return returnRect;
}

cv::Rect expandSquBox(cv::Rect boundingBox,cv::Mat frm,int padding) {
    int center_x = boundingBox.width / 2 + boundingBox.x;
    int center_y = boundingBox.height/ 2 + boundingBox.y;

    int long_side = boundingBox.width > boundingBox.height ? boundingBox.width : boundingBox.height;
    int expand_dist = long_side / 2 + padding;

    int minx = center_x - expand_dist;
    int miny = center_y - expand_dist;
    int maxx = center_x + expand_dist;
    int maxy = center_y + expand_dist;

    int submax = 0;
    
    if (minx < 0) submax = -minx;
    if (miny < 0) submax = -miny > submax ? -miny : submax;
    if (maxx > frm.cols) submax = (maxx - frm.cols) > submax ? (maxx - frm.cols) : submax;
    if (maxy > frm.rows) submax = (maxy - frm.rows) > submax ? (maxy - frm.rows) : submax;

    if (submax > 0) {
        minx -= submax;
        miny -= submax;
        maxx -= submax;
        maxy -= submax;
    }

    cv::Rect returnRect = cv::Rect(minx,miny,maxx-minx,maxy-miny);
    if (returnRect.x < 0)returnRect.x = 0;
    if (returnRect.y < 0)returnRect.y = 0;
    if (returnRect.x+returnRect.width >= frm.cols)returnRect.width = frm.cols-returnRect.x;
    if (returnRect.y+returnRect.height >= frm.rows)returnRect.height = frm.rows-returnRect.y;
    return returnRect;
}

float error_test(Lgvector& lgv1, Lgvector& lgv2) {
    /*
     * Function to check the error between two vectors.
     * t = theta_err = abs(dx1*dx2+dy1*dy2)
     * d = dist_err = exp(-norm((x1-x2), (y1-y2)))
     * Harmonic mean
     * err = 2td/(t+d)
     * 1 for similar, 0 for disimilat 
     */
    float theta_err = abs(lgv1.dx*lgv2.dx+lgv1.dy*lgv2.dy);
    float dist_err = exp((- abs(lgv1.x-lgv2.x) - abs(lgv1.y-lgv2.y))/200 ); // Manhattan dist
    return (2*theta_err*dist_err) / (theta_err+dist_err);
}
} // namespace
