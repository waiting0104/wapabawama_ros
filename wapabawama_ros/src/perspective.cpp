/*
 * perspective.cpp
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */


/*
 * M matric needs to generate by getPerspectiveTransform in Opencv once.
 * This code only transform One point from camera to real world.
 */


#include "perspective.h"


// https://docs.opencv.org/2.4.9/modules/imgproc/doc/geometric_transformations.html#warpperspective
cv::Point2d tf_persp(cv::Point2d src, float* M_, float bx, float by) {
    float x_dst, y_dst, w_dst;
    x_dst = M_[0] * src.x + M_[1] * src.y + M_[2];
    y_dst = M_[3] * src.x + M_[4] * src.y + M_[5];
    w_dst = M_[6] * src.x + M_[7] * src.y + M_[8];
    return cv::Point2d( x_dst/w_dst + bx, y_dst/w_dst + by );
}


cv::Point2d tf_persp_vec(cv::Point2d src, float* M_) {
    cv::Point2d res;
    res.x = M_[0] * src.x + M_[1] * src.y;
    res.y = M_[3] * src.x + M_[4] * src.y;

    res /= cv::sqrt(res.x * res.x + res.y * res.y);
    return res;
}


cv::Point2d tf_persp_v2(cv::Point2d src, float alpha, float cx, float cy, float d) {
    cv::Point2d res;
    res.x = alpha * (src.x - cx);
    res.y = alpha * (cy - src.y) + d;
    return res;
}
cv::Point2d tf_persp_vec_v2(cv::Point2d src) {
    cv::Point2d res;
    if (src.y > 0) {
        res.x =  src.x;
        res.y =  src.y;
    } else {
        res.x = -src.x;
        res.y = -src.y;
    }
    return res;
}
