#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <iostream>
#include <fstream>
#include <Eigen/Dense>

using namespace pinocchio;

int main()
{
    std::string model_path = "../urdf/giraffe_robot.urdf";
    Model model;
    pinocchio::urdf::buildModel(model_path, model);
    Data data(model);

    std::cout << "Model name: " << model.name << " with " << model.nq << " DoF" << std::endl;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

    computeJointPlacement(model, data, q);
    computeFramePlacement(model, data, model.getFrameId("microphone_link"));

    SE3 oMf = data.oMf[model.getFrameId("microphone_link")];
    std::cout << "End-effector position:\n" << oMf.translation().transpose() << std::endl;

    Eigen::MatrixXd J(6, model.nv);
    J = computeFrameJacobian(model, data, q, model.getFrameId("microphone_link"), LOCAL_WORLD_ALIGNED);
    std::cout << "Jacobian:\n" << J << std::endl;

    return 0;
}
