#include <iar_amcl_node/iar_diffdrive_motion_model.hpp>
#include <math.h>
#include <nav2_amcl/angleutils.hpp>
#include <algorithm>
#include <nav2_amcl/pf/pf_pdf.hpp>


namespace iar_amcl
{
    DifferentialMotionModel::DifferentialMotionModel(double alpha1, double alpha2, double alpha3, double alpha4)
    {
        alpha1_ = alpha1;
        alpha2_ = alpha2;
        alpha3_ = alpha3;
        alpha4_ = alpha4;
    }

    void DifferentialMotionModel::odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta)
    {
        // Compute the new sample poses
        pf_sample_set_t * set;

        set = pf->sets + pf->current_set;
        pf_vector_t old_pose = pf_vector_sub(pose, delta);

        // Implement sample_motion_odometry (Prob Rob p 136)
        double delta_rot1, delta_trans, delta_rot2;
        double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
        double delta_rot1_noise, delta_rot2_noise;

        // Avoid computing a bearing from two poses that are extremely near each
        // other (happens on in-place rotation).
        if (sqrt(
            delta.v[1] * delta.v[1] +
            delta.v[0] * delta.v[0]) < 0.01)
        {
            delta_rot1 = 0.0;
        } else {
            delta_rot1 = nav2_amcl::angleutils::angle_diff(
            atan2(delta.v[1], delta.v[0]),
            old_pose.v[2]);
        }
        delta_trans = sqrt(
            delta.v[0] * delta.v[0] +
            delta.v[1] * delta.v[1]);
        delta_rot2 = nav2_amcl::angleutils::angle_diff(delta.v[2], delta_rot1);

        // We want to treat backward and forward motion symmetrically for the
        // noise model to be applied below.  The standard model seems to assume
        // forward motion.
        delta_rot1_noise = std::min(
            fabs(nav2_amcl::angleutils::angle_diff(delta_rot1, 0.0)),
            fabs(nav2_amcl::angleutils::angle_diff(delta_rot1, M_PI)));
        delta_rot2_noise = std::min(
            fabs(nav2_amcl::angleutils::angle_diff(delta_rot2, 0.0)),
            fabs(nav2_amcl::angleutils::angle_diff(delta_rot2, M_PI)));

        for (int i = 0; i < set->sample_count; i++) {
            pf_sample_t * sample = set->samples + i;

            // Sample pose differences
            delta_rot1_hat = nav2_amcl::angleutils::angle_diff(
            delta_rot1,
            pf_ran_gaussian(
                sqrt(
                alpha1_ * delta_rot1_noise * delta_rot1_noise +
                alpha2_ * delta_trans * delta_trans)));
            delta_trans_hat = delta_trans -
            pf_ran_gaussian(
            sqrt(
                alpha3_ * delta_trans * delta_trans +
                alpha4_ * delta_rot1_noise * delta_rot1_noise +
                alpha4_ * delta_rot2_noise * delta_rot2_noise));
            delta_rot2_hat = nav2_amcl::angleutils::angle_diff(
            delta_rot2,
            pf_ran_gaussian(
                sqrt(
                alpha1_ * delta_rot2_noise * delta_rot2_noise +
                alpha2_ * delta_trans * delta_trans)));

            // Apply sampled update to particle pose
            sample->pose.v[0] += delta_trans_hat *
            cos(sample->pose.v[2] + delta_rot1_hat);
            sample->pose.v[1] += delta_trans_hat *
            sin(sample->pose.v[2] + delta_rot1_hat);
            sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
        }
    }
}