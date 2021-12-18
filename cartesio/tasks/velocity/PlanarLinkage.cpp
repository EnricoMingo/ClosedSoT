#include "PlanarLinkage.h"

using namespace XBot::Cartesian::velocity;

std::string task_name(YAML::Node node)
{
    auto distal_link = node["distal_link"].as<std::string>();
    auto base_link = node["base_link"].as<std::string>();
    return "planar_linkage_"+distal_link+base_link;
}

PlanarLinkageImpl::PlanarLinkageImpl(YAML::Node node, Context::ConstPtr context):
    TaskDescriptionImpl (node, context, task_name(node), 2)
{
    _base_link = node["base_link"].as<std::string>();
    _distal_link = node["distal_link"].as<std::string>();
}

const std::string& PlanarLinkageImpl::getBaseLink() const
{
    return _base_link;
}

const std::string& PlanarLinkageImpl::getDistalLink() const
{
    return _distal_link;
}


ClosedSotPlanarLinkageAdapter::ClosedSotPlanarLinkageAdapter(TaskDescription::Ptr ci_task, Context::ConstPtr context):
    OpenSotTaskAdapter(ci_task, context)
{
    _ci_pl = std::dynamic_pointer_cast<PlanarLinkageTask>(ci_task);
    if(!_ci_pl) throw std::runtime_error("Provided task description "
                                            "does not have expected type 'PlanarLinkage'");
    _linkage_factory = std::make_shared<ClosedSoT::tasks::LinkageFactory>();
}

TaskPtr ClosedSotPlanarLinkageAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _closedsot_cc = std::dynamic_pointer_cast<ClosedSoT::tasks::velocity::ClosedChain>(_linkage_factory->createPlanarLinkageTask(const_cast<ModelInterface&>(*_model),
                                                                                                                                 _ci_pl->getDistalLink(), _ci_pl->getBaseLink()));
    _closedsot_cc->cartesian_task->setLambda(_ci_pl->getLambda());

    return _closedsot_cc;
}

void ClosedSotPlanarLinkageAdapter::processSolution(const Eigen::VectorXd& solution)
{
    OpenSotTaskAdapter::processSolution(solution);
}

void ClosedSotPlanarLinkageAdapter::update(double time, double period)
{
    _closedsot_cc->cartesian_task->setLambda(_ci_pl->getLambda());
}

CARTESIO_REGISTER_TASK_PLUGIN(PlanarLinkageImpl, PlanarLinkage)
CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(ClosedSotPlanarLinkageAdapter, PlanarLinkage)

