#ifndef _CARTESIO_CLOSEDCHAIN_H_
#define _CARTESIO_CLOSEDCHAIN_H_

#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Interaction.h>
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <ClosedSoT/tasks/velocity/ClosedChain.h>

using CSoT = ClosedSoT::tasks::velocity::ClosedChain;

namespace XBot {
    namespace Cartesian {
        namespace velocity {

        class ClosedChainTask: public virtual TaskDescription
        {
        public:
            CARTESIO_DECLARE_SMART_PTR(ClosedChainTask)
            virtual const std::string& getBaseLink() const = 0;
            virtual const std::string& getDistalLink() const = 0;
            virtual std::list<unsigned int> getIndices() = 0;
        };

        class ClosedChainImpl: public virtual ClosedChainTask,
                                  public TaskDescriptionImpl
        {
            public:
                CARTESIO_DECLARE_SMART_PTR(ClosedChainImpl)

                const std::string& getBaseLink() const override;
                const std::string& getDistalLink() const override;
                std::list<unsigned int> getIndices() override;

                ClosedChainImpl(YAML::Node node, Context::ConstPtr context);
        private:
                std::string _base_link, _distal_link;
                std::list<unsigned int> _indices;
        };

        class ClosedSotClosedChainAdapter : public OpenSotTaskAdapter
        {

        public:

            ClosedSotClosedChainAdapter(TaskDescription::Ptr ci_task, Context::ConstPtr context);

            virtual TaskPtr constructTask() override;

            virtual void update(double time, double period) override;

            virtual void processSolution(const Eigen::VectorXd& solution) override;

            virtual ~ClosedSotClosedChainAdapter() override = default;

        protected:

            CSoT::Ptr _closedsot_cc;

        private:
            ClosedChainTask::Ptr _ci_cc;

        };

        }
    }
}


#endif
