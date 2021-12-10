#include "ClosedChain.h"

using namespace XBot::Cartesian::velocity;

std::string get_name(YAML::Node node)
{
    auto distal_link = node["distal_link"].as<std::string>();
    auto base_link = node["base_link"].as<std::string>();
    return "closed_chain_"+distal_link+base_link;
}

int get_size(YAML::Node node)
{
    auto v = node["constraint"].as<std::vector<int>>();
    return v.size();
}



ClosedChainImpl::ClosedChainImpl(YAML::Node node, Context::ConstPtr context):
    TaskDescriptionImpl (node, context, get_name(node), get_size(node))
{
    _base_link = node["base_link"].as<std::string>();
    _distal_link = node["distal_link"].as<std::string>();
    std::vector<int> v = node["constraint"].as<std::vector<int>>();

    for(auto i : v)
        _indices.push_back(i);
}

const std::string& ClosedChainImpl::getBaseLink() const
{
    return _base_link;
}

const std::string& ClosedChainImpl::getDistalLink() const
{
    return _distal_link;
}

std::list<unsigned int> ClosedChainImpl::getIndices()
{
    return _indices;
}

ClosedSotClosedChainAdapter::ClosedSotClosedChainAdapter(TaskDescription::Ptr ci_task, Context::ConstPtr context):
    OpenSotTaskAdapter(ci_task, context)
{
    _ci_cc = std::dynamic_pointer_cast<ClosedChainTask>(ci_task);
    if(!_ci_cc) throw std::runtime_error("Provided task description "
                                            "does not have expected type 'ClosedChain'");
}

TaskPtr ClosedSotClosedChainAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _closedsot_cc = std::make_shared<CSoT>(q, const_cast<ModelInterface&>(*_model),
                                            _ci_cc->getDistalLink(), _ci_cc->getBaseLink(), _ci_cc->getIndices());
    _closedsot_cc->cartesian_task->setLambda(_ci_cc->getLambda());

    return _closedsot_cc;
}

void ClosedSotClosedChainAdapter::processSolution(const Eigen::VectorXd& solution)
{
    OpenSotTaskAdapter::processSolution(solution);
}

void ClosedSotClosedChainAdapter::update(double time, double period)
{
    _closedsot_cc->cartesian_task->setLambda(_ci_cc->getLambda());
}

CARTESIO_REGISTER_TASK_PLUGIN(ClosedChainImpl, ClosedChain)
CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(ClosedSotClosedChainAdapter, ClosedChain)

