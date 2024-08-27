#ifndef TRAJECTORYGENERATOR_H_
#define TRAJECTORYGENERATOR_H_

#include <iostream>
#include <cmath>
#include <string>
#include "Eigen/Dense"

class TrajectoryGenerator
{
public:
    TrajectoryGenerator() {};
    virtual ~TrajectoryGenerator() {};

    void init(double dt, double DoF);

    void SetSinusoidalTrajectory(Eigen::VectorXd qf, Eigen::VectorXd qi, double period);
    void SetSinusoidalTrajectory(double qf, double qi, double period);
    void SetInitialValue(double value) { ref_var_(0) = value; }

    Eigen::VectorXd GetRefVar() { return ref_var_; }
    Eigen::VectorXd GetRefVarDot() { return ref_vardot_; }
    Eigen::VectorXd GetRefVarDDot() { return ref_varddot_; }
    double GetRefvar() { return ref_var_(0); }
    double GetRefvardot() { return ref_vardot_(0); }

    double t_;
    Eigen::VectorXd pre_ref_var_, pre_ref_vardot_, pre_ref_varddot_;
    Eigen::VectorXd vect_qf_, vect_qi_;
    double qf_,qi_;
    bool isEnd;

private:
    // void init(double dt);
    Eigen::VectorXd ref_var_, ref_vardot_, ref_varddot_;
    
    // Eigen::VectorXd qf_, qi_;
    double period_, dt_, pi_, dof_;


};

#endif