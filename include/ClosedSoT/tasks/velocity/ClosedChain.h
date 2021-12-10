#ifndef _CLOSEDSOT_CLOSEDCHAIN_H_
#define _CLOSEDSOT_CLOSEDCHAIN_H_

#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/SubTask.h>

namespace ClosedSoT{
namespace tasks {
namespace velocity{

    /**
     * @brief The ClosedChain class implements a closed chain at velocity level
     */
    class ClosedChain: public OpenSoT::Task < Eigen::MatrixXd, Eigen::VectorXd > {
    public:
        typedef std::shared_ptr<ClosedChain> Ptr;

        /**
         * @brief ClosedChain define a closed kinematic loop between distal_link and base_link frames, constraining the directions
         * defined in rowIndices.
         * @param the robot configuration. The Cartesian task will be created so that the task error is zero in position x.
         * @param robot the robot model. Cartesian expects the robot model to be updated externally.
         * @param distal_link the name of the distal link as expressed in the robot urdf
         * @param base_link the name of the base link as expressed in the robot urdf.
         * @param rowIndices between 0 to 5, meaning 0:2 position and 3:5 orientation
         */
        ClosedChain(const Eigen::VectorXd& x,
                    XBot::ModelInterface &robot,
                    const std::string& distal_link,
                    const std::string& base_link,
                    const std::list<unsigned int> rowIndices);

        virtual ~ClosedChain(){}

        /**
         * @brief cartesian_task is used to implement the constraint, this is accessible from outside to change parameters.
         */
        OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_task;
    private:

        /**
         * @brief _sub_task handles interally the constrained directions of the cartesian_task.
         */
        OpenSoT::SubTask::Ptr _sub_task;

        virtual void _update(const Eigen::VectorXd &x);

    };
}
}
}

#endif
