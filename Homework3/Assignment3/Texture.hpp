//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2\imgproc\types_c.h>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols - 1;
        height = image_data.rows - 1;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        // 限制(u, v)坐标范围
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        // 限制(u, v)坐标范围
        u = std::fmin(std::fmax(u, 0), 1);
        v = std::fmin(std::fmax(v, 0), 1);

        auto u_img = u * width, v_img = (1 - v) * height;

        int u_m = u_img;
        int v_m = v_img;
        u_m = (u_img - u_m) > 0.5 ? std::ceil(u_img) : std::floor(u_img);
        v_m = (v_img - v_m) > 0.5 ? std::ceil(v_img) : std::floor(v_img);
        
        auto u00 = image_data.at<cv::Vec3b>(v_m + 0.5, u_m - 0.5);
        auto u01 = image_data.at<cv::Vec3b>(v_m - 0.5, u_m - 0.5);
        auto u10 = image_data.at<cv::Vec3b>(v_m + 0.5, u_m + 0.5);
        auto u11 = image_data.at<cv::Vec3b>(v_m - 0.5, u_m + 0.5);

        float s = u_img + 0.5 - u_m, t = v_m + 0.5 - v_img;
        auto u0 = u00 + s * (u10 - u00);
        auto u1 = u01 + s * (u11 - u01);
        auto color = u0 + t * (u1 - u0);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
