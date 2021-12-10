#include <ClosedSoT/tasks/velocity/ClosedChain.h>
#include <OpenSoT/utils/AutoStack.h>

using namespace ClosedSoT::tasks::velocity;

ClosedChain::ClosedChain(const Eigen::VectorXd &x, XBot::ModelInterface &robot, const std::string& distal_link, const std::string& base_link, const std::list<unsigned int> rowIndices):
    Task("closed_chain_"+distal_link+base_link, x.size())
{
    if(rowIndices.size() > 6)
        throw std::runtime_error("Expected rowIndices size up to 6!");
    for(auto i : rowIndices)
    {
        if(i >= 6)
            throw std::runtime_error("Expected rowIndices between 0 and 5!");
    }

    cartesian_task = std::make_shared<OpenSoT::tasks::velocity::Cartesian>("Cartesian_"+distal_link+base_link, x, robot, distal_link, base_link);
    _sub_task = cartesian_task%rowIndices;

    _update(x);
}

void ClosedChain::_update(const Eigen::VectorXd &x)
{
    _sub_task->update(x); //internally updates as well Cartesian task

    _A = _sub_task->getA();
    _b = _sub_task->getb();
    _W = _sub_task->getWeight();
}
