// Copyright 2018-2021 Alibaba Group
#include <fstream>
#include <map>

#include <yaml-cpp/yaml.h>

#include "common.h"
#include "params_config.h"

ParamsConfig::ParamsConfig()
{
    if (!loadParamsConfig())
    {
        //! image parameters
        img_bound_ = 40;
        sample_img_min_counts_ = 0;

        // PnP
        pnp_max_ransac_iteration_nb_ = 1000000;
        pnp_min_liners_nb_ = 4;
        pnp_min_matches_nb_ = 3;
        // unit is meter
        pnp_ransac_dist_thres_ = 0.01;
        pnp_refine_dist_scale_ = 2;

        // optimization
        reprojection_err_thres_ = 10.0; // pixel
    }
}

ParamsConfig &ParamsConfig::getInstance()
{
    static ParamsConfig instance;
    return instance;
}

bool ParamsConfig::loadParamsConfig()
{
    std::string config_fn = "config.yaml";

    {
        YAML::Node node;
        try
        {
            node = YAML::LoadFile(config_fn);
        }
        catch (YAML::BadFile &e)
        {
            ERROR_STREAM("[ParamsConfig::load] Could not open file: " << config_fn);
            return false;
        }
        catch (YAML::ParserException &e)
        {
            ERROR_STREAM("[ParamsConfig::load] Invalid file format: " << config_fn);
            return false;
        }
        if (node.IsNull())
        {
            ERROR_STREAM("[ParamsConfig::load] Could not open file: " << config_fn);
            return false;
        }

        // image preprocess parameters
        img_bound_ = node["image_bound"].as<int>();
        sample_img_min_counts_ = node["sample_image_min_counts"].as<int>();

        // PnP parameters
        pnp_max_ransac_iteration_nb_ = node["pnp_max_ransac_iteration_nb"].as<int>();
        pnp_min_matches_nb_ = node["pnp_min_matches_nb"].as<int>();
        pnp_min_liners_nb_ = node["pnp_min_liners_nb"].as<int>();
        pnp_ransac_dist_thres_ = node["pnp_ransac_dist_thres"].as<double>();
        pnp_refine_dist_scale_ = node["pnp_refine_dist_scale"].as<double>();

        // optmization parameters
        reprojection_err_thres_ = node["reprojection_err_thres"].as<double>();
    }

    DEBUG_STREAM("[ParamsConfig] Load params_config file!");
    return true;
}
