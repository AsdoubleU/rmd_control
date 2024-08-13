#include "TrajectoryGenerator.h"

void TrajectoryGenerator::init(double dt, double DoF)
{
    pi_ = M_PI;
    t_ = 0.0;
    dt_ = dt;
    dof_ = DoF;

    vect_qf_.setZero(dof_);
    vect_qi_.setZero(dof_);
    ref_var_.setZero(dof_);
    ref_vardot_.setZero(dof_);
    ref_varddot_.setZero(dof_);

    period_ = 0.0;
    isEnd = false;
}

void TrajectoryGenerator::SetSinusoidalTrajectory(Eigen::VectorXd qf, Eigen::VectorXd qi, double period)
{
    // qf : final angle, qi : initial angle
    vect_qf_ = qf;
    vect_qi_ = qi;
    period_ = period;
    t_ += dt_;

    if (t_ >= period_)
    {
        t_ = period_;
        isEnd = true;
    }

    for (size_t i = 0; i < 4; i++)
    {
        ref_var_(i) = (vect_qf_(i) - vect_qi_(i)) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + vect_qi_(i);

        ref_vardot_(i) = (vect_qf_(i) - vect_qi_(i)) * 0.5 * (pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));

        ref_varddot_(i) = (vect_qf_(i) - vect_qi_(i)) * 0.5 * pow((pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));
    }
}

void TrajectoryGenerator::SetSinusoidalTrajectory(double qf, double qi, double period)
{
    // qf : final angle, qi : initial angle
    qf_ = qf;
    qi_ = qi;
    period_ = period;
    t_ += dt_;

    if (t_ >= period_) {
        t_ = period_;
        isEnd = true;
    }
    
    ref_var_(0) = (qf_ - qi_) * 0.5 * (1 - cos(1.0 * pi_ * t_ / period_)) + qi_;
    ref_vardot_(0) = (qf_ - qi_) * 0.5 * (pi_ / period_) * (sin(1.0 * pi_ * t_ / period_));
    ref_varddot_(0) = (qf_ - qi_) * 0.5 * pow((pi_ / period_), 2) * (cos(1.0 * pi_ * t_ / period_));

    // std::cout<<ref_var_<<std::endl;

}
