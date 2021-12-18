#ifndef _CLOSEDSOT_LINKAGEFACTORY_H_
#define _CLOSEDSOT_LINKAGEFACTORY_H_

#include <ClosedSoT/tasks/velocity/ClosedChain.h>

namespace ClosedSoT{
namespace tasks {

    class LinkageFactory
    {
    public:
        typedef std::shared_ptr<LinkageFactory> Ptr;

        LinkageFactory();

        virtual ~LinkageFactory(){}

        OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr createPlanarLinkageTask(XBot::ModelInterface &robot,
                                                                                         const std::string& distal_link,
                                                                                         const std::string& base_link);

    };

}
}

#endif
