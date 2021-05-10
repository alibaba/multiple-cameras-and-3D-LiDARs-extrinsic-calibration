#include "common.h"

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/filesystem.hpp>

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

bool checkAndCreateFolder(const std::string &folder)
{
    try
    {
        if (!boost::filesystem::exists(folder))
        {
            boost::filesystem::create_directories(folder);
            boost::filesystem::permissions(folder, boost::filesystem::all_all);
        }
    }
    catch (boost::filesystem::filesystem_error &e)
    {
        std::cout << __FUNCTION__ << e.what() << " " << folder << std::endl;
        return false;
    }

    return boost::filesystem::exists(folder);
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

    // 制作比size稍微大一点的随机数列(有重复)
    const auto make_size = static_cast<size_t>(size * 1.2);

    // 重复v到size为止
    std::vector<T> v;
    v.reserve(size);
    while (v.size() != size)
    {
        // 依次添加随机整数列(可能有重复)
        while (v.size() < make_size)
        {
            v.push_back(uniform_int_distribution(random_engine));
        }

        // 排序去除重复->去除重复的数列末尾的数组进入unique_end
        std::sort(v.begin(), v.end());
        auto unique_end = std::unique(v.begin(), v.end());

        // 如果v的尺寸太大，unique_end就换成size的长度
        if (size < static_cast<size_t>(std::distance(v.begin(), unique_end)))
        {
            unique_end = std::next(v.begin(), size);
        }

        // 删除重复部分
        v.erase(unique_end, v.end());
    }

    // 因为是升序，所以洗牌
    std::shuffle(v.begin(), v.end(), random_engine);

    return v;
}

template std::vector<int> create_random_array(size_t, int, int);
template std::vector<unsigned int> create_random_array(size_t, unsigned int, unsigned int);
