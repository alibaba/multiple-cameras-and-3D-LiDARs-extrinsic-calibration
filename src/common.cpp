#include "common.h"

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

size_t randomPick(size_t n)
{
    // add the random seed,
    struct timeval time;
    gettimeofday(&time, NULL);
    std::default_random_engine generator(time.tv_usec);
    std::uniform_int_distribution<size_t> distribution(0, n - 1);

    return distribution(generator);
}

std::mt19937 create_random_engine()
{
    std::random_device random_device;
    std::vector<std::uint_least32_t> v(10);
    std::generate(v.begin(), v.end(), std::ref(random_device));
    std::seed_seq seed(v.begin(), v.end());
    return std::mt19937(seed);
}

unsigned int computeIterationsNumberAdaptively(float inlierProbability, int sample_num)
{
    if (inlierProbability < 0.01)
    {
        return (unsigned int)1000000;
    }
    // const float stable_p = 0.99;
    const float unstable_p = 1 - 0.99;
    const float unstable_p_log = std::log10(unstable_p);
    unsigned int N = 0;
    int s = sample_num;

    float outlier_pro = 1 - std::pow(inlierProbability, s);
    N = unstable_p_log / std::log10(outlier_pro);
    return N;
}

std::vector<Eigen::Vector3d> resampleByIndices(const std::vector<unsigned int> &sample_indices,
                                               const std::vector<Eigen::Vector3d> &data_set)
{
    if (sample_indices.empty() || data_set.empty())
    {
        ERROR_STREAM("[resampleByIndices] Empty Input Parameters!");
        return std::vector<Eigen::Vector3d>();
    }
    if (sample_indices.size() > data_set.size())
    {
        ERROR_STREAM("[resampleByIndice] Sample size is greater than dataset!");
        return std::vector<Eigen::Vector3d>();
    }

    std::vector<Eigen::Vector3d> sampled_data;
    sampled_data.reserve(data_set.size());

    for (size_t i = 0; i < sample_indices.size(); ++i)
    {
        auto indice = sample_indices[i];
        if (indice < data_set.size())
        {
            sampled_data.emplace_back(data_set[indice]);
        }
        else
        {
            ERROR_STREAM("[resampleByIndices] Invalid indice!");
            continue;
        }
    }

    if (sampled_data.empty())
    {
        ERROR_STREAM("[resampledByIndice] Empty output!");
    }
    return sampled_data;
}

std::vector<cv::Point2d> resampleByIndices(const std::vector<unsigned int> &sample_indices,
                                           const std::vector<cv::Point2d> &data_set)
{
    if (sample_indices.empty() || data_set.empty())
    {
        ERROR_STREAM("[resampleByIndices] Empty Input Parameters!");
        return std::vector<cv::Point2d>();
    }
    if (sample_indices.size() > data_set.size())
    {
        ERROR_STREAM("[resampleByIndice] Sample size is greater than dataset!");
        return std::vector<cv::Point2d>();
    }

    std::vector<cv::Point2d> sampled_data;
    sampled_data.reserve(data_set.size());
    for (size_t i = 0; i < sample_indices.size(); ++i)
    {
        auto indice = sample_indices[i];
        if (indice < data_set.size())
        {
            sampled_data.emplace_back(data_set[indice]);
        }
        else
        {
            ERROR_STREAM("[resampleByIndices] Invalid indice!");
            continue;
        }
    }

    if (sampled_data.empty())
    {
        ERROR_STREAM("[resampledByIndice] Empty output!");
    }
    return sampled_data;
}

template <typename T>
std::vector<T> create_random_array(const size_t size, const T rand_min, const T rand_max)
{
    assert(rand_min <= rand_max);
    assert(size <= static_cast<size_t>(rand_max - rand_min + 1));

    auto random_engine = create_random_engine();
    std::uniform_int_distribution<T> uniform_int_distribution(rand_min, rand_max);

    // ?????????size??????????????????????????????(?????????)
    const auto make_size = static_cast<size_t>(size * 1.2);

    // ??????v???size??????
    std::vector<T> v;
    v.reserve(size);
    while (v.size() != size)
    {
        // ???????????????????????????(???????????????)
        while (v.size() < make_size)
        {
            v.push_back(uniform_int_distribution(random_engine));
        }

        // ??????????????????->??????????????????????????????????????????unique_end
        std::sort(v.begin(), v.end());
        auto unique_end = std::unique(v.begin(), v.end());

        // ??????v??????????????????unique_end?????????size?????????
        if (size < static_cast<size_t>(std::distance(v.begin(), unique_end)))
        {
            unique_end = std::next(v.begin(), size);
        }

        // ??????????????????
        v.erase(unique_end, v.end());
    }

    // ??????????????????????????????
    std::shuffle(v.begin(), v.end(), random_engine);

    return v;
}

template std::vector<int> create_random_array(size_t, int, int);
template std::vector<unsigned int> create_random_array(size_t, unsigned int, unsigned int);
