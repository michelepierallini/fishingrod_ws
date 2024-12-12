#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>

/* This class implements a Serial Elastic Actuator (SEA)
  It can be used JUST FOR the Softleg Goatleg
*/

namespace gazebo
{
  class SerialElasticPluginFish : public ModelPlugin
  {
    public: 
      void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
      {
        this->model = model;
        if (sdf->HasElement("stiffness"))
          this->stiffness = sdf->Get<double>("stiffness");

        if (sdf->HasElement("damping"))
          this->damping = sdf->Get<double>("damping");

        // Get joint
        if (sdf->HasElement("joint_name"))
        {
          std::string jointName = sdf->Get<std::string>("joint_name");
          this->joint = this->model->GetJoint(jointName);
          if (!this->joint)
            gzerr << "Joint " << jointName << " not found in the model!\n";
        }
        else
        {
          gzerr << "No joint_name specified in the URDF plugin element.\n";
        }

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SerialElasticPluginFish::OnUpdate, this));
      }

      int processJoint(const std::string& jointName) 
      {
        std::string prefix = "Joint_";
        if (jointName.find(prefix) == 0) {
            std::string numberPart = jointName.substr(prefix.length());

            // Convert to an integer
            int jointNumber = std::stoi(numberPart);
            // std::cout << "Joint: " << jointName << ", Number: " << jointNumber << ", Damping: " << damping << ", Stiffness: " << stiffness << std::endl;
            return jointNumber;
          }
          return -1;
      }

      void OnUpdate()
      {
        if (!this->joint)
          return;

        // Get current position and velocity
        double position = this->joint->Position(0);
        double velocity = this->joint->GetVelocity(0);

        int jointNumber = processJoint(this->joint->GetName());

        double scale_K = 2;
        double scale_D = 1;

        if (jointNumber < 10)
        {
          scale_K = 1.5;
          scale_D = 0.75;
        } 
        else 
        {
          scale_K = 1.0;
          scale_D = 0.5;
        }

        double torque = -this->stiffness * scale_K * position - this->damping * scale_D * velocity;

        this->joint->SetForce(0, torque);
      }

    private:
      physics::ModelPtr model;
      physics::JointPtr joint;
      double stiffness = 0.0;  // Default stiffness
      double damping = 0.005;  // Default damping
      event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SerialElasticPluginFish)
}
