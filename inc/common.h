#ifndef __COMMON_H
#define __COMMON_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <functional>
#include <memory>
#include <cassert>
#include <limits>
#include <sys/time.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#define WARN_STREAM(x) std::cerr << "\033[0;33m[WARN] " << x << "\033[0;0m" << std::endl;
#define ERROR_STREAM(x) std::cerr << "\033[1;31m[ERROR] " << x << "\033[0;0m" << std::endl;

#define DEBUG_STREAM(x) std::cout << "\033[0;0m[INFO] " << x << "\033[0;0m" << std::endl;
#define TIMER_STREAM(x) std::cout << "\033[0;32m[TIMER] " << x << "\033[0;0m" << std::endl;

#define DEGREE_2_RAD 0.01745329
#define RAD_2_DEGREE 57.29577951

#define MAX_ANGLE_NUM 8

#define DEBUG_2D_DELAUNY_RESULT 0
#define DEBUG_OPTIMIZED_CAMERA_POSE_REPROJECTION_ERROR 0
#define CERES_OPTIMZE_INTRINSIC 0

struct float2
{
    float x, y;
};
struct float3
{
    float x, y, z;
};
struct double3
{
    double x, y, z;
};
struct uint3
{
    uint32_t x, y, z;
};
struct uint4
{
    uint32_t x, y, z, w;
};


class AccData
{
    public:
    AccData( )
    : a( 0.0 )
    , t( 0.0 )
    {
    }
    AccData( double data, double time )
    : a( data )
    , t( time )
    {
    }

    double a;
    double t;
};

class GyrData
{
    public:
    GyrData( )
    : w( 0.0 )
    , t( 0.0 )
    {
    }
    GyrData( double data, double time )
    : w( data )
    , t( time )
    {
    }

    double w;
    double t;
};

// use bounding box to verify camera pose computed by relocalizer
struct MapBoundingBox
{
    //   bottom   /|  Z  front
    //           /
    //          /
    //         /
    //        /
    // left   ------------------> X  right
    //       |
    //       |
    //  rear |
    //       |
    //       |
    //       |
    //       \/ Y
    //      top
    // unit is meter
    double margin;
    double left_margin;
    double right_margin;
    double front_margin;
    double rear_margin;
    double bottom_margin;
    double top_margin;

    MapBoundingBox(double m = 0.2) : margin(m)
    {
    }

    bool setMapPoints(const std::vector<Eigen::Vector3d> &vMapPoints)
    {
        if (vMapPoints.empty())
        {
            std::cout << "[MapBoundingBox] Empty input map points!\n";
            return false;
        }

        std::vector<double> v_x;
        std::vector<double> v_y;
        std::vector<double> v_z;

        for (size_t i = 0; i < vMapPoints.size(); ++i)
        {
            const Eigen::Vector3d &point = vMapPoints[i];
            v_x.push_back(point[0]);
            v_y.push_back(point[1]);
            v_z.push_back(point[2]);
        }

        std::sort(v_x.begin(), v_x.end());
        std::sort(v_y.begin(), v_y.end());
        std::sort(v_z.begin(), v_z.end());

        double min_x = v_x.at(0);
        double max_x = v_x.back();
        double min_y = v_y.at(0);
        double max_y = v_y.back();
        double min_z = v_z.at(0);
        double max_z = v_z.back();

        left_margin = min_x + margin;
        right_margin = max_x - margin;
        front_margin = max_z - margin;
        rear_margin = min_z + margin;
        top_margin = max_y - margin;
        bottom_margin = min_y + margin;

        std::cout << "[MapBoundingBox] x_min : " << left_margin << "  "
                  << "x_max : " << right_margin << "  "
                  << "z_min : " << rear_margin << "  "
                  << "z_max : " << front_margin << "  "
                  << "y_min : " << bottom_margin << "  "
                  << "y_max : " << top_margin << "\n";

        return true;
    }

    bool beInBoundingBox(const Eigen::Vector3d &cameraCenter)
    {
        if (cameraCenter.hasNaN())
        {
            std::cout << "[MapBoundingBox] Invalid camera center!\n";
            return false;
        }

        if (left_margin < cameraCenter[0] && cameraCenter[0] < right_margin &&
            bottom_margin < cameraCenter[1] && cameraCenter[1] < top_margin &&
            rear_margin < cameraCenter[2] && cameraCenter[2] < front_margin)
        {
            return true;
        }

        return false;
    }

    bool beInBoundingBox(const Eigen::Matrix4d &T_wc)
    {
        if (T_wc.hasNaN())
        {
            std::cout << "[MapBoundingBox] Invalid camera pose!\n";
            return false;
        }

        const Eigen::Vector3d cam_center = T_wc.block<3, 1>(0, 3);
        return beInBoundingBox(cam_center);
    }
};

unsigned int computeIterationsNumberAdaptively(float inlierProbability, int sample_num);

template <typename T>
void FullCombination(T pos, T sample_cnt,
                     T array_size, T sample_nb,
                     T array[], bool visited[],
                     std::vector<std::vector<T>> &output_array)
{
    //????????????sample_nb?????????????????????
    if (sample_cnt == sample_nb)
    {
        std::vector<T> output;
        //        std::cout << "cout :";
        for (T i = 0; i < array_size; i++)
        {
            if (visited[i])
            {
                //                std::cout << array[i] << ' ';
                output.push_back(array[i]);
            }
        }
        output_array.push_back(output);
        //        std::cout << std::endl;
        return;
    }

    //???????????????????????????????????????
    if (pos == array_size)
        return;

    //??????a[pos]???????????????
    if (!visited[pos])
    {
        //??????a[pos]
        visited[pos] = true;
        //???????????????array[pos+1, array_size-1]?????????sample_nb-1??????????????????
        FullCombination(pos + 1, sample_cnt + 1, array_size, sample_nb, array, visited, output_array);
        //??????
        visited[pos] = false;
    }
    //???????????????array[pos+1, array_size-1]?????????sample_nb???????????????
    FullCombination(pos + 1, sample_cnt, array_size, sample_nb, array, visited, output_array);
}

template <typename T>
void FullPermutation(T start, T end, T array[], std::vector<std::vector<T>> &output_array)
{
    //?????????????????????????????????????????????
    if (start == end)
    {
        std::vector<T> output;
        for (T i = 0; i < end; i++)
        {
            //            std::cout << array[i] << ' ';
            output.push_back(array[i]);
        }
        //        std::cout << "\n";
        output_array.push_back(output);

        return;
    }
    for (T i = start; i < end; i++)
    {
        std::swap(array[start], array[i]);                    //??????
        FullPermutation(start + 1, end, array, output_array); //??????????????????a[start+1,...,end-1]????????????
        std::swap(array[i], array[start]);                    //??????
    }
}

/**
 * @brief Read intrinsic file.
 * @param file_name is the intrinsic file_name.
 * @param intrinsic is the camera intrinsic matrix, composed of [fx,fy,cx,cy].
 * @return .
 */
bool readIntrinsicFile(const std::string &file_name, Eigen::Matrix3d &intrinsic);

/**
 * @brief random pick samples out of totalSamples.
 * @param n is the number of random range.
 * @return a sampled number.
 */
size_t randomPick(size_t n);

/**
   * @brief Resample inlier map points by indices.
   * @param sanple_indices is the inlier indices of map points.
   * @param data_set is total map points.
   * @return resampled map points.
   */
std::vector<Eigen::Vector3d> resampleByIndices(const std::vector<unsigned int> &sample_indices,
                                               const std::vector<Eigen::Vector3d> &data_set);

/**
   * @brief Resample inlier 2d keypoints by indices.
   * @param sanple_indices is the inlier indices of 2d keypoints.
   * @param data_set is total 2d keypoints.
   * @return resampled 2d keypoints.
   */
std::vector<cv::Point2d> resampleByIndices(const std::vector<unsigned int> &sample_indices,
                                           const std::vector<cv::Point2d> &data_set);

std::mt19937 create_random_engine();

template <typename T>
std::vector<T> create_random_array(const size_t size, const T rand_min, const T rand_max);

#endif