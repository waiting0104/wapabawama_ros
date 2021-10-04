/*
 * perspective.h
 * Copyright (C) 2021 nvidia <nvidia@nvidia-desktop>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef PERSPECTIVE_H
#define PERSPECTIVE_H

#include <opencv2/opencv.hpp>

const float bias_x = -395;
const float bias_y = -105;


cv::Point2d tf_persp(cv::Point2d, float*, float = bias_x, float = bias_y);
cv::Point2d tf_persp_vec(cv::Point2d, float*);

cv::Point2d tf_persp_v2(cv::Point2d, float, float, float, float);
cv::Point2d tf_persp_vec_v2(cv::Point2d);

#endif /* !PERSPECTIVE_H */
