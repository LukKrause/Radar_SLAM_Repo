//
// Created by Hyungtae Lim on 1/24/22.
// Our code is based on Quatro and TEASER++
// Please refer to following site: https://github.com/url-kaist/Quatro
//

#ifndef NONMINIMAL_SOLVER_H
#define NONMINIMAL_SOLVER_H


#include <unistd.h>
#include <iostream>

//pcl voxelgrid
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "solver/graph.h"
#include "conversion.hpp"

using namespace std;
using namespace pcl;

typedef pcl::PointXYZ PointType;

template<typename T>
void voxelize(
        const boost::shared_ptr<pcl::PointCloud<T> > srcPtr, boost::shared_ptr<pcl::PointCloud<T> > dstPtr,
        double voxelSize) {
    static pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(srcPtr);
    voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxel_filter.filter(*dstPtr);
}

template<typename T>
void voxelize(
        pcl::PointCloud<T> &src, boost::shared_ptr<pcl::PointCloud<T> > dstPtr,
        double voxelSize) {
    static pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(src);
    voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxel_filter.filter(*dstPtr);
}

namespace orora {
    class NonMinimalSolver {

    public:
        using InlierSetPairs = tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>;

        /** \brief Empty constructor. */
        NonMinimalSolver() {};

        /** \brief Empty destructor */
        ~NonMinimalSolver() {}

        struct RegistrationSolution {
            bool            valid = true;
            double          scale = 1.0;
            Eigen::Vector3d translation;
            Eigen::Matrix3d rotation;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        RegistrationSolution solution_;

        enum class ROTATION_ESTIMATION_ALGORITHM {
            GNC_TLS = 0,
            FGR     = 1,
        };
        /**
        * Enum representing the type of graph-based inlier selection algorithm to use
        *
        * PMC_EXACT: Use PMC to find exact clique from the inlier graph
        * PMC_HEU: Use PMC's heuristic finder to find approximate max clique
        * KCORE_HEU: Use k-core heuristic to select inliers
        * NONE: No inlier selection
        */
        enum class INLIER_SELECTION_MODE {
            PMC_EXACT = 0,
            PMC_HEU   = 1,
            KCORE_HEU = 2,
            NONE      = 3,
        };

        /**
        * Enum representing the formulation of the TIM graph after maximum clique filtering.
        *
        * CHAIN: formulate TIMs by only calculating ttable(run_pmc_pruninghe TIMs for consecutive measurements
        * COMPLETE: formulate a fully connected TIM graph
        */
        enum class INLIER_GRAPH_FORMULATION {
            CHAIN    = 0,
            COMPLETE = 1,
        };

        struct Params {
            // A bound on the noise of each provided measurement.
            double                        noise_bound                   = 0.3;
            /**
            * Square of the ratio between acceptable noise and noise bound. Usually set to 1.
            */
            double                        cbar2                         = 1;
            /**
            * Whether the scale is known. If set to False, the solver assumes no scale differences
            * between the src and dst points. If set to True, the solver will first solve for scale.
            *
            * When the solver does not estimate scale, it solves the registration problem much faster.
            */
            bool                          estimate_scaling              = true;
            // Which algorithm to use to estimate rotations.
            ROTATION_ESTIMATION_ALGORITHM rotation_estimation_algorithm = ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
            /**
            * Factor to multiple/divide the control parameter in the GNC algorithm.
            *
            * For FGR: the algorithm divides the control parameter by the factor every iteration.
            * For GNC-TLS: the algorithm multiples the control parameter by the factor every iteration.
            */
            double                        rotation_gnc_factor           = 1.4;
            // Maximum iterations allowed for the GNC rotation estimators.
            size_t                        rotation_max_iterations       = 100;
            /**
            * Cost threshold for the GNC rotation estimators.
            *
            * For FGR / GNC-TLS algorithm, the cost thresholds represent different meanings.
            * For FGR: the cost threshold compares with the computed cost at each iteration
            * For GNC-TLS: the cost threshold compares with the difference between costs of consecutive
            * iterations.
            */
            double                        rotation_cost_threshold       = 1e-6;

            // Type of TIM graph given to GNC rotation solver
            INLIER_GRAPH_FORMULATION rotation_tim_graph        = INLIER_GRAPH_FORMULATION::CHAIN;
            // brief Type of the inlier selection
            INLIER_SELECTION_MODE    inlier_selection_mode     = INLIER_SELECTION_MODE::PMC_HEU; //PMC_EXACT -> PMC_HEU
            /**
            * \brief The threshold ratio for determining whether to skip max clique and go straightly to
            * GNC rotation estimation. Set this to 1 to always use exact max clique selection, 0 to always
            * skip exact max clique selection.
            * \attention Note that the use_max_clique option takes precedence. In other words, if
            * use_max_clique is set to false, then kcore_heuristic_threshold will be ignored. If
            * use_max_clique is set to true, then the following will happen: if the max core number of the
            * inlier graph is lower than the kcore_heuristic_threshold as a percentage of the total nodes
            * in the inlier graph, then the code will preceed to call the max clique finder. Otherwise, the
            * graph will be directly fed to the GNC rotation solver.
            *
            */
            double                   kcore_heuristic_threshold = 0.5;
            // \deprecated Use inlier_selection_mode instead. Set this to true to enable max clique inlier selection, false to skip it.
            bool                     use_max_clique            = true;
            // Use inlier_selection_mode instead  * Set this to false to enable heuristic only max clique finding.
            bool                     max_clique_exact_solution = true;
            //Timelimit on running the max clique algorithm (in seconds).
            double                   max_clique_time_limit     = 3600;
        };
        double               noise_bound_;
        double               noise_bound_radial_;
        double               noise_bound_tangential_;

        Params getParams() { return params_; }

        void setParams(Params params) { params_ = params; }

        void setInputSource(const Eigen::MatrixXd &cloudEigen) {
            int N = cloudEigen.cols();
            src_matched.resize(3, N);
            src_matched.topRows(2)    = cloudEigen.topRows(2);
            src_matched.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);
        }

        inline void setInputTarget(const Eigen::MatrixXd &cloudEigen) {
            int N = cloudEigen.cols();
            tgt_matched.resize(3, N);
            tgt_matched.topRows(2)    = cloudEigen.topRows(2);
            tgt_matched.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);
        }

        Eigen::Matrix<double, 3, Eigen::Dynamic> computeTIMs(
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &v,
                Eigen::Matrix<int, 2, Eigen::Dynamic> *map) {

            auto                                     N = v.cols();
            // std::cout<<"\033[1;32m"<< "N:"<<N<<"\033[0m"<<std::endl;
            // std::cout<<"\033[1;32m"<< "N2:"<<N * (N - 1) / 2<<"\033[0m"<<std::endl;
            Eigen::Matrix<double, 3, Eigen::Dynamic> vtilde(3, N * (N - 1) / 2);
            map->resize(2, N * (N - 1) / 2);
            // std::cout<<"\033[1;32m"<< "Debug 1-3"<<"\033[0m"<<std::endl;
#pragma omp parallel for default(none) shared(N, v, vtilde, map)
            for (size_t i = 0; i < N - 1; i++) {
                // Calculate some important indices
                // For each measurement, we compute the TIMs between itself and all the measurements after it.
                // For examples:
                // i=0: add N-1 TIMs // i=1: add N-2 TIMs // etc.. // i=k: add N-1-k TIMs
                // And by arithmatic series, we can get the starting index of each segment be:
                // k*N - k*(k+1)/2
                size_t segment_start_idx = i * N - i * (i + 1) / 2;
                size_t segment_cols      = N - 1 - i;

                // calculate TIM
                Eigen::Matrix<double, 3, 1>              m    = v.col(i);
                Eigen::Matrix<double, 3, Eigen::Dynamic> temp = v - m * Eigen::MatrixXd::Ones(1, N);

                // concatenate to the end of the tilde vector
                vtilde.middleCols(segment_start_idx, segment_cols) = temp.rightCols(segment_cols);

                // populate the index map
                Eigen::Matrix<int, 2, Eigen::Dynamic> map_addition(2, N);
                for (size_t                           j          = 0; j < N; ++j) {
                    map_addition(0, j) = i;
                    map_addition(1, j) = j;
                }
                map->middleCols(segment_start_idx, segment_cols) = map_addition.rightCols(segment_cols);
            }
            return vtilde;
        }

        double solveForScale(
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &v1,
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &v2) {
            scale_inliers_mask_.resize(1, v1.cols());
            solveForScale(v1, v2, &(solution_.scale),
                          &scale_inliers_mask_);     //ScaleInliersSelector의 solveForScale임 //scale_solver_-> 뺌
            return solution_.scale;
        }

        void solveForScale(
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &src,
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &dst,
                double *scale,
                Eigen::Matrix<bool, 1, Eigen::Dynamic> *inliers) {
            // We assume no scale difference between the two vectors of points.
            *scale = 1;

            Eigen::Matrix<double, 1, Eigen::Dynamic> v1_dist            =
                                                             src.array().square().colwise().sum().array().sqrt();
            Eigen::Matrix<double, 1, Eigen::Dynamic> v2_dist            =
                                                             dst.array().square().colwise().sum().array().sqrt();
            double                                   beta               = 2 * params_.noise_bound *
                                                                          sqrt(params_.cbar2);                             ////cbar2_, noise_bound_
            // A pair-wise correspondence is an inlier if it passes the following two tests:
            // 1. dst / src is within maximum allowed error
            // 2. src / dst is within maximum allowed error
            Eigen::Matrix<double, 1, Eigen::Dynamic> alphas_forward     = beta * v1_dist.cwiseInverse();
            Eigen::Matrix<double, 1, Eigen::Dynamic> raw_scales_forward = v2_dist.array() / v1_dist.array();
            Eigen::Matrix<bool, 1, Eigen::Dynamic>   inliers_forward    =
                                                             (raw_scales_forward.array() - *scale).array().abs() <=
                                                             alphas_forward.array();

            Eigen::Matrix<double, 1, Eigen::Dynamic> alphas_reverse     = beta * v2_dist.cwiseInverse();
            Eigen::Matrix<double, 1, Eigen::Dynamic> raw_scales_reverse = v1_dist.array() / v2_dist.array();
            Eigen::Matrix<bool, 1, Eigen::Dynamic>   inliers_reverse    =
                                                             (raw_scales_reverse.array() - *scale).array().abs() <=
                                                             alphas_reverse.array();

            // element-wise AND using component-wise product (Eigen 3.2 compatible)
            *inliers = inliers_forward.cwiseProduct(inliers_reverse);
        }

        Eigen::Matrix3d solveForRotation(
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &v1,
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &v2,
                const Eigen::Matrix4d initial_guess) {

            rotation_inliers_mask_.resize(1, v1.cols());

            Eigen::Matrix2d                          rotation_2d;
            Eigen::Matrix<double, 2, Eigen::Dynamic> src_2d;
            Eigen::Matrix<double, 2, Eigen::Dynamic> dst_2d;
            // XY Coordinates for calculate yaw
            src_2d.resize(2, v1.cols());
            dst_2d.resize(2, v2.cols());
            src_2d = v1.topRows(2);
            dst_2d = v2.topRows(2);

            Eigen::Matrix3d rot_yaw = Eigen::Matrix3d::Identity();
            rotation_2d = Eigen::Matrix2d::Identity();
            solveForRotation2D(src_2d, dst_2d, initial_guess.block<2, 2>(0, 0), &rotation_2d,
                               &rotation_inliers_mask_);      //rotation_solver_-> 뺌
            rot_yaw.block<2, 2>(0, 0) = rotation_2d;
            solution_.rotation = rot_yaw;

            return solution_.rotation;
        }

        void solveForRotation2D(
                const Eigen::Matrix<double, 2, Eigen::Dynamic> &src,
                const Eigen::Matrix<double, 2, Eigen::Dynamic> &dst,
                const Eigen::Matrix2d initial_rot,
                Eigen::Matrix2d *rotation,
                Eigen::Matrix<bool, 1, Eigen::Dynamic> *inliers) {
            assert(rotation);                 // make sure R is not a nullptr
            assert(src.cols() == dst.cols()); // check dimensions of input data
            assert(params_.rotation_gnc_factor >
                   1);   // make sure mu will increase        gnc_factor -> rotation_gnc_factor
            assert(params_.noise_bound != 0); // make sure noise sigma is not zero


            if (inliers) {
                assert(inliers->cols() == src.cols());
            }

            std::string   costPath, estWeightPath, estRotPath;
            std::ofstream costTxt, weightTxt, rotTxt;

            // Prepare some variables
            size_t match_size = src.cols(); // number of correspondences

            double mu = 1; // arbitrary starting mu

            double prev_cost = std::numeric_limits<double>::infinity();
            cost_ = std::numeric_limits<double>::infinity();
            //  double noise_bound_sq = std::pow(params_.noise_bound, 2);
            double noise_bound_sq = std::pow(params_.noise_bound, 2);
            if (noise_bound_sq < 1e-16) {
                noise_bound_sq = 1e-2;
            }

            Eigen::Matrix<double, 2, Eigen::Dynamic> diffs(2, match_size);
            Eigen::Matrix<double, 1, Eigen::Dynamic> weights(1, match_size);
            weights.setOnes(1, match_size);
            Eigen::Matrix<double, 1, Eigen::Dynamic> residuals_sq(1, match_size);

            /*** Calculating Initial weights ***/
            // Calculate residuals squared
//        diffs        = (dst - (initial_rot) * src).array().square();
            *rotation = svdRot2d(src, dst, rot_initial_weights_);
            // Calculate residuals squared
            diffs        = (dst - (*rotation) * src).array().square();
            residuals_sq = diffs.colwise().sum();
//        std::cout << "\033[1;31mQ: " << residuals_sq.maxCoeff() << "\033[0m" << std::endl;
            static double EPSILON      = 0.00000000000001;
            // Initialize rule for mu
            double        max_residual = residuals_sq.maxCoeff();
            // For stop motion
            if (max_residual < noise_bound_sq) {
                noise_bound_sq = 0.01; // Forcely set noise bound into 0.1 => 0.1 * 0.1
            }
            mu                         = 1 / (2 * max_residual / noise_bound_sq - 1);
            // Degenerate case: mu = -1 because max_residual is very small
            // i.e., little to none noise
            if (mu <= 0) {
                TEASER_DEBUG_INFO_MSG(
                        "GNC-TLS terminated because maximum residual at initialization is very small.");
                return;
            }
            // Fix R and solve for weights in closed form
            double th1 = (mu + 1) / mu * noise_bound_sq;
            double th2 = mu / (mu + 1) * noise_bound_sq;
            cost_ = 0;

            for (size_t j = 0; j < match_size; ++j) {
                // Also calculate cost in this loop
                // Note: the cost calculated is using the previously solved weights
                cost_ += weights(j) * residuals_sq(j);

                if (residuals_sq(j) >= th1) {
                    weights(j) = 0;
                } else if (residuals_sq(j) <= th2) {
                    weights(j) = 1;
                } else {
                    weights(j) = sqrt(noise_bound_sq * mu * (mu + 1) / residuals_sq(j) + EPSILON) - mu;
                    assert(weights(j) >= 0 && weights(j) <= 1);
                }
            }
            mu = mu * params_.rotation_gnc_factor;
            /*********************************************/

            // Loop for performing GNC-TLS
            for (size_t i = 0;
                 i < params_.rotation_max_iterations; ++i) {   //max_iterations  = >rotation_max_iterations
                // Fix weights and perform SVD rotation estimation
                *rotation = svdRot2d(src, dst, weights);
                // Calculate residuals squared
                diffs        = (dst - (*rotation) * src).array().square();
                residuals_sq = diffs.colwise().sum();

                // Fix R and solve for weights in closed form
                double th1 = (mu + 1) / mu * noise_bound_sq;
                double th2 = mu / (mu + 1) * noise_bound_sq;
                cost_ = 0;

                // For ablation study
                static bool use_weighting = false;
                double      max_w         = 0.0;
                for (size_t j             = 0; j < match_size; ++j) {
                    // Also calculate cost in this loop
                    // Note: the cost calculated is using the previously solved weights
                    cost_ += weights(j) * residuals_sq(j);

                    if (residuals_sq(j) >= th1) {
                        weights(j) = 0;
                    } else if (residuals_sq(j) <= th2) {
                        weights(j) = 1;
                        if (use_weighting) {
                            weights(j) *= rot_initial_weights_(0, j);
                            max_w = max(max_w, weights(j));
                        }
                    } else {
                        weights(j) = sqrt(noise_bound_sq * mu * (mu + 1) / residuals_sq(j)) - mu;
                        if (use_weighting) {
                            weights(j) *= rot_initial_weights_(0, j);
                            max_w = max(max_w, weights(j));
                        }
                        assert(weights(j) >= 0 && weights(j) <= 1);
                    }
                }
                // Normlization
                if (use_weighting) {
                    for (int j = 0; j < match_size; ++j) {
                        weights(j) /= max_w;
                    }
                }
                // Calculate cost
                double      cost_diff     = std::abs(cost_ - prev_cost);

                // Increase mu
                mu        = mu * params_.rotation_gnc_factor;   //gnc_factor -> rotation_gnc_factor
                prev_cost = cost_;

                if (cost_diff < params_.rotation_cost_threshold) {
                    TEASER_DEBUG_INFO_MSG("GNC-TLS solver terminated due to cost convergence.");
                    TEASER_DEBUG_INFO_MSG("Cost diff: " << cost_diff);
                    TEASER_DEBUG_INFO_MSG("Iterations: " << i);
                    break;
                }
            }

            if (inliers) {
                for (size_t i = 0; i < weights.cols(); ++i) {
                    (*inliers)(0, i) = weights(0, i) >= 0.4;
                }
            }
        }

        Eigen::Vector3d solveForTranslation(
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &v1,
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &v2,
                bool using_median_selection = false) {

            translation_inliers_mask_.resize(1, v1.cols());
            solveForTranslation(v1, v2, &(solution_.translation), &translation_inliers_mask_, using_median_selection);

            return solution_.translation;
        }

        double calcCov11(const double c, const double s, const double r_sq, const double var_r, const double var_t) {
            return c * c * var_r + s * s * r_sq * var_t;
        }

        double calcCov22(const double c, const double s, const double r_sq, const double var_r, const double var_t) {
            return s * s * var_r + c * c * r_sq * var_t;
        }

        void updateAnisotropicNoiseBound(const Eigen::VectorXd src, const Eigen::VectorXd dst,
                                         const double var_r, const double var_t, Eigen::Vector2d &alpha_col) {
            double ang_src = atan2(src(1), src(0));
            double ang_dst = atan2(dst(1), dst(0));
            double c_s     = cos(ang_src);
            double s_s     = sin(ang_src);
            double c_d     = cos(ang_dst);
            double s_d     = sin(ang_dst);
            double r_sqr_s = src.squaredNorm();
            double r_sqr_d = dst.squaredNorm();

            double cov_src_11 = calcCov11(c_s, s_s, r_sqr_s, var_r, var_t);
            double cov_src_22 = calcCov22(c_s, s_s, r_sqr_s, var_r, var_t);
            double cov_dst_11 = calcCov11(c_d, s_d, r_sqr_d, var_r, var_t);
            double cov_dst_22 = calcCov22(c_d, s_d, r_sqr_d, var_r, var_t);
            alpha_col(0) = sqrt(cov_src_11 + cov_dst_11);
            alpha_col(1) = sqrt(cov_src_22 + cov_dst_22);
        }

        void solveForTranslation(
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &src,
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &dst,
                Eigen::Vector3d *translation,
                Eigen::Matrix<bool, 1, Eigen::Dynamic> *inliers,
                bool using_median_selection) {

            assert(src.cols() == dst.cols());
            if (inliers) {
                assert(inliers->cols() == src.cols());
            }
            // Raw translation
            Eigen::Matrix<double, 3, Eigen::Dynamic> raw_translation = dst - src;
            // Error bounds for each measurements
            int                                      N               = src.cols();

            // Estimate x, y, and z component of translation: perform TLS on each row
            *inliers = Eigen::Matrix<bool, 1, Eigen::Dynamic>::Ones(1, N);
            Eigen::Matrix<bool, 1, Eigen::Dynamic> inliers_temp(1, N);

            double var_r = noise_bound_radial_ * noise_bound_radial_;
            double var_t = noise_bound_tangential_ * noise_bound_tangential_;

            Eigen::Matrix<double, 2, Eigen::Dynamic> alphas = Eigen::MatrixXd::Ones(2, N);
            // Why only x and y (i.e. 2)?: Because radar sensor is 2D!
//        for (size_t i = 0; i < 2; ++i) {
            for (int                                 j      = 0; j < N; ++j) {
                double ang_src = atan2(src(1, j), src(0, j));
                double ang_dst = atan2(dst(1, j), dst(0, j));
                double c_s     = cos(ang_src);
                double s_s     = sin(ang_src);
                double c_d     = cos(ang_dst);
                double s_d     = sin(ang_dst);
                double r_sqr_s = src.col(j).squaredNorm();
                double r_sqr_d = dst.col(j).squaredNorm();

                double cov_src_11 = calcCov11(c_s, s_s, r_sqr_s, var_r, var_t);
                double cov_src_22 = calcCov22(c_s, s_s, r_sqr_s, var_r, var_t);
                double cov_dst_11 = calcCov11(c_d, s_d, r_sqr_d, var_r, var_t);
                double cov_dst_22 = calcCov22(c_d, s_d, r_sqr_d, var_r, var_t);
                alphas(0, j) = sqrt(cov_src_11 + cov_dst_11);
                alphas(1, j) = sqrt(cov_src_22 + cov_dst_22);
            }
            estimate(raw_translation.row(0), alphas.row(0), &((*translation)(0)), &inliers_temp,
                     using_median_selection);
            *inliers = (*inliers).cwiseProduct(inliers_temp);
            estimate(raw_translation.row(1), alphas.row(1), &((*translation)(1)), &inliers_temp,
                     using_median_selection);
            *inliers = (*inliers).cwiseProduct(inliers_temp);
        }


        void estimate(
                const Eigen::RowVectorXd &X,
                const Eigen::RowVectorXd &ranges,
                double *estimate,
                Eigen::Matrix<bool, 1, Eigen::Dynamic> *inliers,
                bool using_median_selection = false) {
            // check input parameters
            bool dimension_inconsistent = (X.rows() != ranges.rows()) || (X.cols() != ranges.cols());
            if (inliers) {
                dimension_inconsistent |= ((inliers->rows() != 1) || (inliers->cols() != ranges.cols()));
            }
            bool only_one_element = (X.rows() == 1) && (X.cols() == 1);
            assert(!dimension_inconsistent);
            assert(!only_one_element); // TODO: admit a trivial solution

            int                                 N = X.cols();
            std::vector<std::pair<double, int>> h;
            for (size_t                         i = 0; i < N; ++i) {
                h.push_back(std::make_pair(X(i) - ranges(i), i + 1));
                h.push_back(std::make_pair(X(i) + ranges(i), -i - 1));
            }

            // ascending order
            std::sort(h.begin(), h.end(),
                      [](std::pair<double, int> a, std::pair<double, int> b) { return a.first < b.first; });

            // calculate weights
            Eigen::RowVectorXd weights = ranges.array().square();
            weights = weights.array().inverse();
            int                nr_centers      = 2 * N;
            Eigen::RowVectorXd x_hat           = Eigen::MatrixXd::Zero(1, nr_centers);
            Eigen::RowVectorXd x_cost          = Eigen::MatrixXd::Zero(1, nr_centers);
            Eigen::RowVectorXi set_cardinality = Eigen::MatrixXi::Zero(1, nr_centers);

            double ranges_inverse_sum     = ranges.sum();
            double dot_X_weights          = 0;
            double dot_weights_consensus  = 0;
            int    consensus_set_cardinal = 0;
            double sum_xi                 = 0;
            double sum_xi_square          = 0;

            // To save outputs
            static int component_idx = 0;
            component_idx = component_idx % 3;

            std::string   h_path, estimate_path, idx_path;
            std::ofstream h_txt, estimate_txt, idx_txt;
            // if (SAVE_RESULTS) {
            //     h_path =
            //             (boost::format("%s/%04d_h_%d.txt") % SAVE_ABS_PATH % COUNT_EXECUTION % component_idx).str();

            //     estimate_path = (boost::format("%s/%04d_estimates_%d.txt") % SAVE_ABS_PATH % COUNT_EXECUTION %
            //                      component_idx)
            //             .str();

            //     idx_path = (boost::format("%s/%04d_final_idx_%d.txt") % SAVE_ABS_PATH % COUNT_EXECUTION %
            //                 component_idx)
            //             .str();
            //     estimate_txt.open(estimate_path);
            //     estimate_txt.close();

            //     idx_txt.open(estimate_path);
            //     idx_txt.close();

            //     h_txt.open(h_path);
            //     for (int ijk = 0; ijk < h.size(); ++ijk) {
            //         h_txt << h[ijk].first << " " << h[ijk].second << "\n";
            //     }
            //     h_txt.close();
            // }
            for (size_t i = 0; i < nr_centers; ++i) {

                int idx     = int(std::abs(h.at(i).second)) - 1; // Indices starting at 1
                int epsilon = (h.at(i).second > 0) ? 1 : -1;

                consensus_set_cardinal += epsilon;
                dot_weights_consensus += epsilon * weights(idx);
                dot_X_weights += epsilon * weights(idx) * X(idx); // X(idx): v_ij in my paper
                ranges_inverse_sum -= epsilon * ranges(idx);      // handling truncated loss!
                sum_xi += epsilon * X(idx);
                sum_xi_square += epsilon * X(idx) * X(idx);

                if (using_median_selection) {
                    // To set median values
                    set_cardinality(i) = consensus_set_cardinal;
                }
                x_hat(i)    = dot_X_weights / dot_weights_consensus;
                // sum_xi_square: already includes consensus_set_cardinal
                double residual =
                               consensus_set_cardinal * x_hat(i) * x_hat(i) + sum_xi_square - 2 * sum_xi * x_hat(i);
                x_cost(i) = residual + ranges_inverse_sum;
            }

            size_t min_idx;
            x_cost.minCoeff(&min_idx);

            double median;
            if (using_median_selection) {
                int n_card = set_cardinality(min_idx);
                std::cout << "\033[1;32m[Median Selection] num: " << n_card << "\033[0m" << std::endl;
                if (n_card > 0) {
                    std::vector<double> candidates;
                    candidates.reserve(set_cardinality(min_idx));
                    for (int j = 0; j < n_card; ++j) {
                        int idx = int(std::abs(h.at(min_idx - j).second)) - 1;
                        assert(idx >= 0);
                        candidates.push_back(X(idx));
                    }
                    sort(candidates.begin(), candidates.end());
                    median = static_cast<double>(candidates[candidates.size() / 2 - 1] +
                                                 candidates[candidates.size() / 2]) / 2.0;
                }
            }
            double estimate_temp;
            if (using_median_selection) {
                estimate_temp = median;
            } else {
                estimate_temp = x_hat(min_idx);
            }

            if (estimate) {
                // update estimate output if it's not nullptr
                *estimate = estimate_temp;
            }
            if (inliers) {
                // update inlier output if it's not nullptr
                *inliers = (X.array() - estimate_temp).array().abs() <= ranges.array();
            }
            component_idx++;
        }

        double cost_;

        void reset(const Params &params, const double noise_bound_radial, const double noise_bound_tangential) {
            params_                 = params;
            noise_bound_            = params.noise_bound;
            noise_bound_radial_     = noise_bound_radial;
            noise_bound_tangential_ = noise_bound_tangential;

            // Clear member variables
            max_clique_.clear();
            rotation_inliers_.clear();
            translation_inliers_.clear();
            inlier_graph_.clear();
        }

        void computeTransformation(const Eigen::Matrix4d &initial_guess, Eigen::Matrix4d &output) {
            // It is important
            // Once estimation fails, it returns identity matrix
            output = Eigen::Matrix4d::Identity();

            src_tims_ = computeTIMs(src_matched, &src_tims_map_);
            dst_tims_ = computeTIMs(tgt_matched, &dst_tims_map_);
            solveForScale(src_tims_, dst_tims_);

            // Calculate Maximum Clique
            if (params_.inlier_selection_mode != INLIER_SELECTION_MODE::NONE) { //0,1,2 -> default 1

                inlier_graph_.populateVertices(src_matched.cols());
                for (size_t i = 0; i < scale_inliers_mask_.cols(); ++i) {
                    if (scale_inliers_mask_(0, i)) {
                        inlier_graph_.addEdge(src_tims_map_(0, i), src_tims_map_(1, i));
                    }
                }

                teaser::MaxCliqueSolver::Params clique_params;
                if (params_.inlier_selection_mode == INLIER_SELECTION_MODE::PMC_EXACT) {
                    clique_params.solver_mode = teaser::MaxCliqueSolver::CLIQUE_SOLVER_MODE::PMC_EXACT;
                } else if (params_.inlier_selection_mode == INLIER_SELECTION_MODE::PMC_HEU) {
                    clique_params.solver_mode = teaser::MaxCliqueSolver::CLIQUE_SOLVER_MODE::PMC_HEU;
                } else {
                    clique_params.solver_mode = teaser::MaxCliqueSolver::CLIQUE_SOLVER_MODE::KCORE_HEU;
                }

                clique_params.time_limit                = params_.max_clique_time_limit;
                clique_params.kcore_heuristic_threshold = params_.kcore_heuristic_threshold;

                teaser::MaxCliqueSolver clique_solver(clique_params);
                max_clique_ = clique_solver.findMaxClique(inlier_graph_);

                std::sort(max_clique_.begin(), max_clique_.end());
//            std::copy(max_clique_.begin(), max_clique_.end(), std::ostream_iterator<int>(std::cout, " "));
//            std::cout << std::endl;
                if (max_clique_.size() <= 1) {
                    std::cout << "\033[1;31m[Warning] Too few " << max_clique_.size() << " cliques exist!!!\033[0m" << std::endl;
                    cin.ignore();
                    solution_.valid = false;
                    return;
                }
            }

            // Calculate new measurements & TIMs based on max clique inliers
            double   max_w = 0.0;
            if (params_.rotation_tim_graph == INLIER_GRAPH_FORMULATION::CHAIN) {
                // ==============>
                // chain graph
                // std::cout << "\033[1;36m# of maximum cliques: " << max_clique_.size() << "\033[0m\n";
                num_maxclique_ = max_clique_.size();
                rot_initial_weights_.resize(1, max_clique_.size());
                pruned_src_tims_.resize(3, max_clique_.size());
                pruned_dst_tims_.resize(3, max_clique_.size());
                src_tims_map_rotation_.resize(2, max_clique_.size());
                dst_tims_map_rotation_.resize(2, max_clique_.size());
                for (size_t i  = 0; i < max_clique_.size(); ++i) {
                    const auto &root = max_clique_[i];
                    int        leaf;
                    if (i != max_clique_.size() - 1) {
                        leaf = max_clique_[i + 1];
                    } else {
                        leaf = max_clique_[0];
                    }
                    pruned_src_tims_.col(i)      = src_matched.col(leaf) - src_matched.col(root);
                    pruned_dst_tims_.col(i)      = tgt_matched.col(leaf) - tgt_matched.col(root);
                    // populate the TIMs map
                    dst_tims_map_rotation_(0, i) = leaf;
                    dst_tims_map_rotation_(1, i) = root;
                    src_tims_map_rotation_(0, i) = leaf;
                    src_tims_map_rotation_(1, i) = root;
                    // Update initial rotation_weights
                    double r_sq_l = tgt_matched.col(leaf).squaredNorm();
                    double r_sq_r = tgt_matched.col(root).squaredNorm();
                    rot_initial_weights_(0, i) = 1.0 / sqrt(r_sq_l + r_sq_r);
                    max_w = max(max_w, rot_initial_weights_(0, i));
                }
            }
            // Weights are normalized
            for (int k     = 0; k < rot_initial_weights_.cols(); ++k) {
                rot_initial_weights_(0, k) /= max_w;
            }
            // Remove scaling for rotation estimation
            pruned_dst_tims_ *= (1 / solution_.scale);
            // Update GNC rotation solver's noise bound with the new information
            // Note: this implicitly assumes that rotation_solver_'s noise bound
            // is set to the original noise bound of the measurements.
//        auto params = getParams();            //rotation_solver_-> 뺌
//        params.noise_bound *= (2 / solution_.scale);
//        setParams(params);                     //rotation_solver_-> 뺌

            // Solve for rotation
            solveForRotation(pruned_src_tims_, pruned_dst_tims_, initial_guess);

            rotation_inliers_.clear();

            // Save indices of inlier TIMs from GNC rotation estimation
            for (size_t i = 0; i < rotation_inliers_mask_.cols(); ++i) {
                if (i == 0) {
                    // Check (N-1)-th and 0-th
                    if (rotation_inliers_mask_(0, rotation_inliers_mask_.cols() - 1) &&
                        rotation_inliers_mask_(0, i)) {
                        rotation_inliers_.emplace_back(i);
                    }
                } else {
                    // Check i-1th and i-th
                    if (rotation_inliers_mask_(0, i - 1) && rotation_inliers_mask_(0, i)) {
                        rotation_inliers_.emplace_back(i);
                    }
                }
            }
            num_rot_inliers_ = rotation_inliers_.size();
            int                                      N_R = rotation_inliers_.size();
            Eigen::Matrix<double, 3, Eigen::Dynamic> rotation_pruned_src;
            Eigen::Matrix<double, 3, Eigen::Dynamic> rotation_pruned_dst;

            int N_MC = max_clique_.size();
            rotation_pruned_src.resize(3, N_MC);
            rotation_pruned_dst.resize(3, N_MC);
            for (size_t i = 0; i < N_MC; ++i) {
                rotation_pruned_src.col(i) = src_matched.col(max_clique_[i]);
                rotation_pruned_dst.col(i) = tgt_matched.col(max_clique_[i]);
            }

            solveForTranslation(solution_.scale * solution_.rotation * rotation_pruned_src,
                                rotation_pruned_dst);

            // Find the final inliers
            translation_inliers_ = findNonzero<bool>(translation_inliers_mask_);

            // final_inliers: Indices order of matched pairs
            // which corresponds to `input_` and `target_`
            // I.e. indices of src and dst
            final_inliers_.clear();
            final_inliers_.reserve(translation_inliers_.size());
            // Note that `params_.using_rot_inliers_when_estimating_cote` == false exhibits better results

            for (const auto &idx: translation_inliers_) {
                final_inliers_.push_back(max_clique_[idx]);
            }

            solution_.valid = true;
            solution_.translation(2) = 0.0;

            output.block<3, 3>(0, 0)    = solution_.rotation;
            output.topRightCorner(3, 1) = solution_.translation;
        }

        pcl::PointCloud<PointType> setInliers(
                const Eigen::Matrix<double, 3, Eigen::Dynamic> &raw,
                const vector<int> &idx_inliers) {
            int                        N = idx_inliers.size();
            pcl::PointCloud<PointType> inliers;
            inliers.reserve(N);

            for (const int idx: idx_inliers) {
                PointType p;
                p.x = raw(0, idx);
                p.y = raw(1, idx);
                p.z = raw(2, idx);

                inliers.points.emplace_back(p);
            }
        }

        inline Eigen::Matrix2d svdRot2d(const Eigen::Matrix<double, 2, Eigen::Dynamic> &X,
                                        const Eigen::Matrix<double, 2, Eigen::Dynamic> &Y,
                                        const Eigen::Matrix<double, 1, Eigen::Dynamic> &W) {
            // Assemble the correlation matrix H = X * Y'
            Eigen::Matrix2d H = X * W.asDiagonal() * Y.transpose();

            Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix2d                   U = svd.matrixU();
            Eigen::Matrix2d                   V = svd.matrixV();

            if (U.determinant() * V.determinant() < 0) {
                V.col(1) *= -1;
            }

            return V * U.transpose();
        }

        template<class T>
        inline std::vector<int> findNonzero(Eigen::Matrix<T, 1, Eigen::Dynamic> mask) {
            std::vector<int> result;
            for (size_t      i = 0; i < mask.cols(); ++i) {
                if (mask(i)) {
                    result.emplace_back(i);
                }
            }
            return result;
        }

        InlierSetPairs getMaxCliques() {
            auto source_max_clique = setInliers(src_matched, max_clique_);
            auto target_max_clique = setInliers(tgt_matched, max_clique_);
            return {source_max_clique, target_max_clique};
        }

        InlierSetPairs getFinalInliers() {
            auto source_inliers = setInliers(src_matched, final_inliers_);
            auto target_inliers = setInliers(tgt_matched, final_inliers_);
            return {source_inliers, target_inliers};
        }

        vector<int> getFinalInliersIndices() {
            return final_inliers_;
        }

        int getNumRotaionInliers() {
            return num_rot_inliers_;
        }

        int getNumMaxCliqueInliers() {
            return num_maxclique_;
        }

    protected:
        //Params.
        Params params_;

        int num_rot_inliers_;
        int num_maxclique_;

        // Inlier graph
        Eigen::Matrix<double, 1, Eigen::Dynamic> rot_initial_weights_;
        teaser::Graph                            inlier_graph_;
        // Inlier Binary Vectors
        Eigen::Matrix<bool, 1, Eigen::Dynamic>   rotation_inliers_mask_;
        Eigen::Matrix<bool, 1, Eigen::Dynamic>   translation_inliers_mask_;
        Eigen::Matrix<bool, 1, Eigen::Dynamic>   scale_inliers_mask_;
        // TIMs
        // TIMs used for scale estimation/pruning
        Eigen::Matrix<double, 3, Eigen::Dynamic> src_tims_;
        Eigen::Matrix<double, 3, Eigen::Dynamic> dst_tims_;
        // TIMs used for rotation estimation
        Eigen::Matrix<double, 3, Eigen::Dynamic> pruned_src_tims_;
        Eigen::Matrix<double, 3, Eigen::Dynamic> pruned_dst_tims_;
        // Pruned data Eigen 2 Pcl
        pcl::PointCloud<PointType>               pcl_pruned_src_;
        pcl::PointCloud<PointType>               pcl_pruned_dst_;

        // TIM maps
        // for scale estimation
        Eigen::Matrix<int, 2, Eigen::Dynamic> src_tims_map_;
        Eigen::Matrix<int, 2, Eigen::Dynamic> dst_tims_map_;
        // for rotation estimation
        Eigen::Matrix<int, 2, Eigen::Dynamic> src_tims_map_rotation_;
        Eigen::Matrix<int, 2, Eigen::Dynamic> dst_tims_map_rotation_;
        // Max clique vector
        std::vector<int>                      max_clique_;
        // Inliers after rotation estimation
        std::vector<int>                      rotation_inliers_;
        // Inliers after translation estimation (final inliers)
        std::vector<int>                      translation_inliers_;
        std::vector<int>                      final_inliers_;

        /** \brief XYZ fields offset. */
        std::size_t x_idx_offset_, y_idx_offset_, z_idx_offset_;

        /** \brief Normal fields offset. */
        std::size_t nx_idx_offset_, ny_idx_offset_, nz_idx_offset_;

        /** \brief Internal check whether source dataset has normals or not. */
        bool source_has_normals_;
        /** \brief Internal check whether target dataset has normals or not. */
        bool target_has_normals_;

        /** \brief Checks for whether estimators and rejectors need various data */
        bool need_source_blob_, need_target_blob_;

        Eigen::Matrix<double, 3, Eigen::Dynamic> src_matched;
        Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_matched;

        typedef std::vector<Eigen::VectorXf> Feature; //public

    private:

    };

    inline void setParams(
            double noise_bound_of_each_measurement, double cbar2,
            double estimating_scale, int num_max_iter, double control_parameter_for_gnc,
            double rot_cost_thr, NonMinimalSolver::Params &params) {
        params.noise_bound                   = noise_bound_of_each_measurement;
        params.cbar2                         = cbar2;
        params.estimate_scaling              = estimating_scale;
        params.rotation_max_iterations       = num_max_iter;
        params.rotation_gnc_factor           = control_parameter_for_gnc;
        params.rotation_estimation_algorithm = NonMinimalSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
        params.rotation_cost_threshold       = rot_cost_thr;

//    params.inlier_selection_mode = NonMinimalSolver::INLIER_SELECTION_MODE::PMC_HEU;
        params.inlier_selection_mode = NonMinimalSolver::INLIER_SELECTION_MODE::PMC_EXACT;
    }
}

#endif //NONMINIMAL_SOLVER_H