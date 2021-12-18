#ifndef _CARTESIO_PLANARLINKAGE_H_
#define _CARTESIO_PLANARLINKAGE_H_

#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <ClosedSoT/tasks/LinkageFactory.h>

using CSoT = ClosedSoT::tasks::velocity::ClosedChain;

namespace XBot {
    namespace Cartesian {
        namespace velocity {

        class PlanarLinkageTask: public virtual TaskDescription
        {
        public:
            CARTESIO_DECLARE_SMART_PTR(PlanarLinkageTask)
            virtual const std::string& getBaseLink() const = 0;
            virtual const std::string& getDistalLink() const = 0;
        };

        class PlanarLinkageImpl: public virtual PlanarLinkageTask,
                                  public TaskDescriptionImpl
        {
            public:
                CARTESIO_DECLARE_SMART_PTR(PlanarLinkageImpl)

                const std::string& getBaseLink() const override;
                const std::string& getDistalLink() const override;

                PlanarLinkageImpl(YAML::Node node, Context::ConstPtr context);
        private:
                std::string _base_link, _distal_link;
        };

        class ClosedSotPlanarLinkageAdapter : public OpenSotTaskAdapter
        {

        public:

            ClosedSotPlanarLinkageAdapter(TaskDescription::Ptr ci_task, Context::ConstPtr context);

            virtual TaskPtr constructTask() override;

            virtual void update(double time, double period) override;

            virtual void processSolution(const Eigen::VectorXd& solution) override;

            virtual ~ClosedSotPlanarLinkageAdapter() override = default;

        protected:

            CSoT::Ptr _closedsot_cc;

        private:
            ClosedSoT::tasks::LinkageFactory::Ptr _linkage_factory;
            PlanarLinkageTask::Ptr _ci_pl;

        };

        }
    }
}


#endif
