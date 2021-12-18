#include <ClosedSoT/tasks/LinkageFactory.h>

using namespace ClosedSoT::tasks;

LinkageFactory::LinkageFactory()
{

}

OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr LinkageFactory::createPlanarLinkageTask(XBot::ModelInterface &robot,
                                                               const std::string& distal_link,
                                                               const std::string& base_link)
{
    Eigen::MatrixXd J(6, robot.getJointNum());
    J.setZero();
    robot.getRelativeJacobian(distal_link, base_link, J);

    std::list<unsigned int> constrained_dofs;
    //for now we consider only position
    for(unsigned int i = 0; i < 3; ++i)
    {
        if(!J.row(i).isZero())
            constrained_dofs.push_back(i);
    }

    if(constrained_dofs.size() == 3)
        throw std::runtime_error("Overconstrained! Can not create a Planar linkage!");

    Eigen::VectorXd q(robot.getJointNum());
    q.setZero();
    return std::make_shared<velocity::ClosedChain>(q, robot, distal_link, base_link, constrained_dofs);
}
