// Copyright 2018-2021 Alibaba Group
#ifndef PARAMS_CONFIG_H_
#define PARAMS_CONFIG_H_

#include <iostream>
#include <string>
#include <vector>

// Class manage all params
class ParamsConfig
{
public:
    static ParamsConfig &getInstance();

    static unsigned int GetImageBound() { return getInstance().img_bound_; }
    static unsigned int GetMinSampleImageCounts() { return getInstance().sample_img_min_counts_; }

    static unsigned int GetPnPMaxRansacNum() { return getInstance().pnp_max_ransac_iteration_nb_; }
    static unsigned int GetPnPMinInlierNum() { return getInstance().pnp_min_liners_nb_; }
    static unsigned int GetPnPMinMatchesNum() { return getInstance().pnp_min_matches_nb_; }
    static double GetPnPRansacDistThres() { return getInstance().pnp_ransac_dist_thres_; }
    static double GetPnPRefineDistScale() { return getInstance().pnp_refine_dist_scale_; }

    static double GetMinReprojectionErrThreshold() { return getInstance().reprojection_err_thres_; }

private:
    ParamsConfig();

    ParamsConfig(const ParamsConfig &);
    ParamsConfig &operator=(const ParamsConfig &);

    // load deafuat setting files in application folder
    bool loadParamsConfig();

    //! image preprocess parameters
    unsigned int img_bound_;
    // min sample image numebr
    unsigned int sample_img_min_counts_;

    //! PnP parameters
    // max ransac number
    unsigned int pnp_max_ransac_iteration_nb_;
    // min matches number feed into pnp
    unsigned int pnp_min_matches_nb_;
    // min inliers number
    unsigned int pnp_min_liners_nb_;
    // epipolar distance scale during pnp refinement
    double pnp_refine_dist_scale_;
    // epipoloar distance threhsold during pnp ransac
    double pnp_ransac_dist_thres_;

    //! optimization parameters
    double reprojection_err_thres_;
};

#endif // PARAMS_CONFIG_H_
