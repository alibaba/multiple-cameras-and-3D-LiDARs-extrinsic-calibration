#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

// projection/observation data structure
struct Projection
{
    Projection(int i, int j) : frameIndex(i), pointIndex(j)
    {
    }
    // frame index
    int frameIndex;
    // 2d feature index
    int pointIndex;
};

// landmark data structure
struct Track
{
    Track()
    {
        IsStable = false;
        id = 0;
    }
    cv::Point3d position;
    std::vector<Projection> projections;
    bool IsStable;
    int id;
};