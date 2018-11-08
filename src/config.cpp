/*****************************************************************************
**      Stereo VO and SLAM by combining point and line segment features     **
******************************************************************************
**                                                                          **
**  Copyright(c) 2016-2018, Ruben Gomez-Ojeda, University of Malaga         **
**  Copyright(c) 2016-2018, David Zuñiga-Noël, University of Malaga         **
**  Copyright(c) 2016-2018, MAPIR group, University of Malaga               **
**                                                                          **
**  This program is free software: you can redistribute it and/or modify    **
**  it under the terms of the GNU General Public License (version 3) as     **
**  published by the Free Software Foundation.                              **
**                                                                          **
**  This program is distributed in the hope that it will be useful, but     **
**  WITHOUT ANY WARRANTY; without even the implied warranty of              **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            **
**  GNU General Public License for more details.                            **
**                                                                          **
**  You should have received a copy of the GNU General Public License       **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.   **
**                                                                          **
*****************************************************************************/

#include <config.h>
#include<opencv2/core/core.hpp>
using namespace std;

Config::Config()
{
    img_width = 0;
    img_height = 0;

    // kf decision (SLAM) parameters
    min_entropy_ratio     = 0.85;
    max_kf_t_dist         = 5.0;
    max_kf_r_dist         = 15.0;

    // StVO-PL options
    // -----------------------------------------------------------------------------------------------------
    has_points         = true;      // true if using points
    has_lines          = true;      // true if using line segments
    use_fld_lines      = false;     // true if using FLD detector
    lr_in_parallel     = true;      // true if detecting and matching features in parallel
    pl_in_parallel     = true;      // true if detecting points and line segments in parallel
    best_lr_matches    = true;      // true if double-checking the matches between the two images
    adaptative_fast    = true;      // true if using adaptative fast_threshold
    use_motion_model   = false;     // true if using constant motion model

    // Tracking parameters
    // -----------------------------------------------------------------------------------------------------
    // Point features
    max_dist_epip     = 1.0;        // max. epipolar distance in pixels
    min_disp          = 1.0;        // min. disparity (avoid points in the infinite)
    min_ratio_12_p    = 0.9;        // min. ratio between the first and second best matches

    // Line segment features
    line_sim_th       = 0.75;       // threshold for cosine similarity
    stereo_overlap_th = 0.75;
    f2f_overlap_th    = 0.75;
    min_line_length   = 0.025;      // min. line length (relative to img size)
    line_horiz_th     = 0.1;        // parameter to avoid horizontal lines (pixels)
    min_ratio_12_l    = 0.9;        // parameter to avoid outliers in line matching
    ls_min_disp_ratio = 0.7;        // min ratio between min(disp)/max(disp) for a LS

    // Adaptative FAST parameters
    fast_min_th       = 5;          // min. value for FAST threshold
    fast_max_th       = 50;         // max. value for FAST threshold
    fast_inc_th       = 5;          // base increment for the FAST threshold
    fast_feat_th      = 50;         // base number of features to increase/decrease FAST threshold
    fast_err_th       = 0.5;        // threshold for the optimization error

    // Optimization parameters
    // -----------------------------------------------------------------------------------------------------
    homog_th         = 1e-7;        // avoid points in the infinite
    min_features     = 10;          // min. number of features to perform StVO
    max_iters        = 5;           // max. number of iterations in the first stage of the optimization
    max_iters_ref    = 10;          // max. number of iterations in the refinement stage
    min_error        = 1e-7;        // min. error to stop the optimization
    min_error_change = 1e-7;        // min. error change to stop the optimization
    inlier_k         = 4.0;         // factor to discard outliers before the refinement stage

    // Feature detection parameters
    // -----------------------------------------------------------------------------------------------------
    matching_strategy = 3;
    matching_s_ws     = 10;
    matching_f2f_ws   = 3;

    // ORB detector
    orb_nfeatures    = 1200;
    orb_scale_factor = 1.2;
    orb_nlevels      = 4;
    orb_edge_th      = 19;
    orb_wta_k        = 2;            // was set to 4
    orb_score        = 1;            // 0 - HARRIS  |  1 - FAST
    orb_patch_size   = 31;
    orb_fast_th      = 20;           // default FAST threshold
    // LSD parameters
    lsd_nfeatures    = 300;          // set to 0 if keeping all lines
    lsd_refine       = 0;
    lsd_scale        = 1.2;
    lsd_sigma_scale  = 0.6;
    lsd_quant        = 2.0;
    lsd_ang_th       = 22.5;
    lsd_log_eps      = 1.0;
    lsd_density_th   = 0.6;
    lsd_n_bins       = 1024;


        // SLAM parameters
    // -----------------------------------------------------------------------------------------------------
    fast_matching         = true;       // allow for the fast matching (window-based) of the map features
    has_refinement        = false;      // refine the pose between keyframes (disabled as it is also performed by the LBA)
    mutithread_slam       = true;       // if true the system runs with both the VO, LBA and LC in parallel threads

    // lm numbers and errors
    min_lm_obs            = 5;          // min number of observations for a landmark to be considered as inlier
    max_common_fts_kf     = 0.9;        // max number of common features for a keyframe to be considered redundant (disabled)

    max_kf_epip_p         = 1.0;
    max_kf_epip_l         = 1.0;

    max_lm_3d_err         = 0.1;
    max_lm_dir_err        = 0.1;
    max_point_point_error = 0.1;
    max_point_line_error  = 0.1;
    max_dir_line_error    = 0.1;

    // graphs parameters
    min_lm_ess_graph      = 150;
    min_lm_cov_graph      = 75;
    min_kf_local_map      = 3;

    // LBA
    lambda_lba_lm         = 0.00001;
    lambda_lba_k          = 10.0;
    max_iters_lba         = 15;

    // Loop closure
    vocabulary_p          = "";
    vocabulary_l          = "";

    lc_mat                = 0.5;
    lc_res                = 1.5;
    lc_unc                = 0.01;
    lc_inl                = 0.3;
    lc_trs                = 1.5;
    lc_rot                = 35.0;

    max_iters_pgo         = 100;
    lc_kf_dist            = 50;
    lc_kf_max_dist        = 50;
    lc_nkf_closest        = 4;
    lc_inlier_ratio       = 30.0;

    min_pt_matches        = 10;
    min_ls_matches        = 6;
    kf_inlier_ratio       = 30.0;
}

Config::~Config(){}

Config& Config::getInstance()
{
    static Config instance; // Instantiated on first use and guaranteed to be destroyed
    return instance;
}


void Config::loadFromFile( const string &strSettingPath )
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    Config::imgWidth() = fSettings["Camera.width"];
    Config::imgHeight() = fSettings["Camera.height"];

    Config::minEntropyRatio() = fSettings["min_entropy_ratio"]; //loadSafe(config, "min_entropy_ratio", Config::minEntropyRatio()); 
    Config::maxKFTDist() = fSettings["max_kf_t_dist"]; //loadSafe(config, "max_kf_t_dist", Config::maxKFTDist());
    Config::maxKFRDist() = fSettings["max_kf_r_dist"]; //loadSafe(config, "max_kf_r_dist", Config::maxKFRDist());

    Config::hasPoints() = bool(int(fSettings["has_points"])); //loadSafe(config, "has_points", Config::hasPoints());
    Config::hasLines() = bool(int(fSettings["has_lines"])); //loadSafe(config, "has_lines", Config::hasLines());
    Config::useFLDLines() = bool(int(fSettings["use_fld_lines"])); //loadSafe(config, "use_fld_lines", Config::useFLDLines());
    Config::lrInParallel() = bool(int(fSettings["lr_in_parallel"])); //loadSafe(config, "lr_in_parallel", Config::lrInParallel());
    Config::plInParallel() = bool(int(fSettings["pl_in_parallel"])); //loadSafe(config, "pl_in_parallel", Config::plInParallel());
    Config::bestLRMatches() = bool(int(fSettings["best_lr_matches"])); //loadSafe(config, "best_lr_matches", Config::bestLRMatches());
    Config::adaptativeFAST() = bool(int(fSettings["adaptative_fast"])); //loadSafe(config, "adaptative_fast", Config::adaptativeFAST());
    Config::useMotionModel() = bool(int(fSettings["use_motion_model"])); //loadSafe(config, "use_motion_model", Config::useMotionModel());

    Config::maxDistEpip() = fSettings["max_dist_epip"]; //loadSafe(config, "max_dist_epip", Config::maxDistEpip());
    Config::minDisp() = fSettings["min_disp"];  //loadSafe(config, "min_disp", Config::minDisp());
    Config::minRatio12P() = fSettings["min_ratio_12_p"]; //loadSafe(config, "min_ratio_12_p", Config::minRatio12P());

    Config::lineSimTh() = fSettings["line_sim_th"]; //loadSafe(config, "line_sim_th", Config::lineSimTh());
    Config::stereoOverlapTh() = fSettings["stereo_overlap_th"]; //loadSafe(config, "stereo_overlap_th", Config::stereoOverlapTh());
    Config::f2fOverlapTh() = fSettings["f2f_overlap_th"];  //loadSafe(config, "f2f_overlap_th", Config::f2fOverlapTh());
    Config::minLineLength() = fSettings["min_line_length"]; //loadSafe(config, "min_line_length", Config::minLineLength());
    Config::lineHorizTh() = fSettings["line_horiz_th"];  //loadSafe(config, "line_horiz_th", Config::lineHorizTh());
    Config::minRatio12L() = fSettings["min_ratio_12_l"]; //loadSafe(config, "min_ratio_12_l", Config::minRatio12L());
    Config::lsMinDispRatio() = fSettings["ls_min_disp_ratio"]; //loadSafe(config, "ls_min_disp_ratio", Config::lsMinDispRatio());

    Config::fastMinTh() = fSettings["fast_min_th"]; //loadSafe(config, "fast_min_th", Config::fastMinTh());
    Config::fastMaxTh() = fSettings["fast_max_th"]; //loadSafe(config, "fast_max_th", Config::fastMaxTh());
    Config::fastIncTh() = fSettings["fast_inc_th"]; //loadSafe(config, "fast_inc_th", Config::fastIncTh());
    Config::fastFeatTh() = fSettings["fast_feat_th"]; //loadSafe(config, "fast_feat_th", Config::fastFeatTh());
    Config::fastErrTh() = fSettings["fast_err_th"]; //loadSafe(config, "fast_err_th", Config::fastErrTh());

    Config::rgbdMinDepth() = fSettings["rgbd_min_depth"]; //loadSafe(config, "rgbd_min_depth", Config::rgbdMinDepth());
    Config::rgbdMaxDepth() = fSettings["rgbd_max_depth"]; //loadSafe(config, "rgbd_max_depth", Config::rgbdMaxDepth());

    Config::homogTh() = fSettings["homog_th"]; //loadSafe(config, "homog_th", Config::homogTh());
    Config::minFeatures() = fSettings["min_features"]; //loadSafe(config, "min_features", Config::minFeatures());
    Config::maxIters() = fSettings["max_iters"]; //loadSafe(config, "max_iters", Config::maxIters());
    Config::maxItersRef() = fSettings["max_iters_ref"]; //loadSafe(config, "max_iters_ref", Config::maxItersRef());
    Config::minError() = fSettings["min_error"]; //loadSafe(config, "min_error", Config::minError());
    Config::minErrorChange() = fSettings["min_error_change"]; //loadSafe(config, "min_error_change", Config::minErrorChange());
    Config::inlierK() = fSettings["inlier_k"]; //loadSafe(config, "inlier_k", Config::inlierK());

    Config::matchingStrategy() = fSettings["matching_strategy"]; //loadSafe(config, "matching_strategy", Config::matchingStrategy());
    Config::matchingSWs() = fSettings["matching_s_ws"]; //loadSafe(config, "matching_s_ws", Config::matchingSWs());
    Config::matchingF2FWs() = fSettings["matching_f2f_ws"]; //loadSafe(config, "matching_f2f_ws", Config::matchingF2FWs());

    Config::orbNFeatures() = fSettings["orb_nfeatures"]; //loadSafe(config, "orb_nfeatures", Config::orbNFeatures());
    Config::orbScaleFactor() = fSettings["orb_scale_factor"]; //loadSafe(config, "orb_scale_factor", Config::orbScaleFactor());
    Config::orbNLevels() = fSettings["orb_nlevels"]; //loadSafe(config, "orb_nlevels", Config::orbNLevels());
    Config::orbEdgeTh() = fSettings["orb_edge_th"]; //loadSafe(config, "orb_edge_th", Config::orbEdgeTh());
    Config::orbWtaK() = fSettings["orb_wta_k"]; //loadSafe(config, "orb_wta_k", Config::orbWtaK());
    Config::orbScore() = fSettings["orb_score"]; //loadSafe(config, "orb_score", Config::orbScore());
    Config::orbPatchSize() = fSettings["orb_patch_size"]; //loadSafe(config, "orb_patch_size", Config::orbPatchSize());
    Config::orbFastTh() = fSettings["orb_fast_th"]; //loadSafe(config, "orb_fast_th", Config::orbFastTh());

    Config::lsdNFeatures() = fSettings["lsd_nfeatures"]; //loadSafe(config, "lsd_nfeatures", Config::lsdNFeatures());
    Config::lsdRefine() = fSettings["lsd_refine"]; //loadSafe(config, "lsd_refine", Config::lsdRefine());
    Config::lsdScale() = fSettings["lsd_scale"]; //loadSafe(config, "lsd_scale", Config::lsdScale());
    Config::lsdSigmaScale() = fSettings["lsd_sigma_scale"]; //loadSafe(config, "lsd_sigma_scale", Config::lsdSigmaScale());
    Config::lsdQuant() = fSettings["lsd_quant"]; //loadSafe(config, "lsd_quant", Config::lsdQuant());
    Config::lsdAngTh() = fSettings["lsd_ang_th"]; //loadSafe(config, "lsd_ang_th", Config::lsdAngTh());
    Config::lsdLogEps() = fSettings["lsd_log_eps"]; //loadSafe(config, "lsd_log_eps", Config::lsdLogEps());
    Config::lsdDensityTh() = fSettings["lsd_density_th"]; //loadSafe(config, "lsd_density_th", Config::lsdDensityTh());
    Config::lsdNBins() = fSettings["lsd_n_bins"]; //loadSafe(config, "lsd_n_bins", Config::lsdNBins());



    Config::minLMObs() = fSettings["min_lm_obs"]; //loadSafe(config, "min_lm_obs", SlamConfig::minLMObs());
    Config::maxCommonFtsKF() = fSettings["max_common_fts_kf"]; //loadSafe(config, "max_common_fts_kf", SlamConfig::maxCommonFtsKF());

    Config::maxKFEpipP() = fSettings["max_kf_epip_p"]; //loadSafe(config, "max_kf_epip_p", SlamConfig::maxKFEpipP());
    Config::maxKFEpipL() = fSettings["max_kf_epip_l"]; //loadSafe(config, "max_kf_epip_l", SlamConfig::maxKFEpipL());

    Config::maxLM3DErr() = fSettings["max_lm_3d_err"]; //loadSafe(config, "max_lm_3d_err", SlamConfig::maxLM3DErr());
    Config::maxLMDirErr() = fSettings["max_lm_dir_err"]; //loadSafe(config, "max_lm_dir_err", SlamConfig::maxLMDirErr());
    Config::maxPointPointError() = fSettings["max_point_point_error"]; //loadSafe(config, "max_point_point_error", SlamConfig::maxPointPointError());
    Config::maxPointLineError() = fSettings["max_point_line_error"]; //loadSafe(config, "max_point_line_error", SlamConfig::maxPointLineError());
    Config::maxDirLineError() = fSettings["max_dir_line_error"]; //loadSafe(config, "max_dir_line_error", SlamConfig::maxDirLineError());

    Config::minLMEssGraph() = fSettings["min_lm_ess_graph"]; //loadSafe(config, "min_lm_ess_graph", SlamConfig::minLMEssGraph());
    Config::minLMCovGraph() = fSettings["min_lm_cov_graph"]; //loadSafe(config, "min_lm_cov_graph", SlamConfig::minLMCovGraph());
    Config::minKFLocalMap() = fSettings["min_kf_local_map"]; //loadSafe(config, "min_kf_local_map", SlamConfig::minKFLocalMap());

    Config::lambdaLbaLM() = fSettings["lambda_lba_lm"]; //loadSafe(config, "lambda_lba_lm", SlamConfig::lambdaLbaLM());
    Config::lambdaLbaK() = fSettings["lambda_lba_k"]; //loadSafe(config, "lambda_lba_k", SlamConfig::lambdaLbaK());
    Config::maxItersLba() = fSettings["max_iters_lba"]; //loadSafe(config, "max_iters_lba", SlamConfig::maxItersLba());

    // Config::dbowVocP() = fSettings["vocabulary_p"]; //loadSafe(config, "vocabulary_p", SlamConfig::dbowVocP());
    // Config::dbowVocL() = fSettings["vocabulary_l"]; //loadSafe(config, "vocabulary_l", SlamConfig::dbowVocL());

    Config::lcMat() = fSettings["lc_mat"]; //loadSafe(config, "lc_mat", SlamConfig::lcMat());
    Config::lcRes() = fSettings["lc_res"]; //loadSafe(config, "lc_res", SlamConfig::lcRes());
    Config::lcUnc() = fSettings["lc_unc"]; //loadSafe(config, "lc_unc", SlamConfig::lcUnc());
    Config::lcInl() = fSettings["lc_inl"]; //loadSafe(config, "lc_inl", SlamConfig::lcInl());
    Config::lcTrs() = fSettings["lc_trs"]; //loadSafe(config, "lc_trs", SlamConfig::lcTrs());
    Config::lcRot() = fSettings["lc_rot"]; //loadSafe(config, "lc_rot", SlamConfig::lcRot());

    Config::maxItersPGO() = fSettings["max_iters_pgo"]; //loadSafe(config, "max_iters_pgo", SlamConfig::maxItersPGO());
    Config::lcKFDist() = fSettings["lc_kf_dist"]; //loadSafe(config, "lc_kf_dist", SlamConfig::lcKFDist());
    Config::lcKFMaxDist() = fSettings["lc_kf_max_dist"]; //loadSafe(config, "lc_kf_max_dist", SlamConfig::lcKFMaxDist());
    Config::lcNKFClosest() = fSettings["lc_nkf_closest"]; //loadSafe(config, "lc_nkf_closest", SlamConfig::lcNKFClosest());
    Config::lcInlierRatio() = fSettings["lc_inlier_ratio"]; //loadSafe(config, "lc_inlier_ratio", SlamConfig::lcInlierRatio());

    Config::minPointMatches() = fSettings["min_pt_matches"]; //loadSafe(config, "min_pt_matches", SlamConfig::minPointMatches());
    Config::minLineMatches() = fSettings["min_ls_matches"]; //loadSafe(config, "min_ls_matches", SlamConfig::minLineMatches());
    Config::kfInlierRatio() = fSettings["kf_inlier_ratio"]; //loadSafe(config, "kf_inlier_ratio", SlamConfig::kfInlierRatio());

    Config::fastMatching() = bool(int(fSettings["fast_matching"])); //loadSafe(config, "fast_matching", SlamConfig::fastMatching());
    Config::hasRefinement() = bool(int(fSettings["has_refinement"])); //loadSafe(config, "has_refinement", SlamConfig::hasRefinement());
    Config::multithreadSLAM() = bool(int(fSettings["mutithread_slam"])); //loadSafe(config, "mutithread_slam", SlamConfig::multithreadSLAM());
}
