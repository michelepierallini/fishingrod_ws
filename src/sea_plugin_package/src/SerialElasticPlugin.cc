#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>

/* This class implements a Serial Elastic Actuator (SEA)
  It can be used also for the Mulinex robot and the Softleg Goatleg
*/

namespace gazebo
{
  class SerialElasticPlugin : public ModelPlugin
  {
    public: 
      void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
      {
        this->model = model;
        // Parse stiffness (K) and damping (D) parameters
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
            std::bind(&SerialElasticPlugin::OnUpdate, this));
      }

      void OnUpdate()
      {
        if (!this->joint)
          return;

        // Get current position and velocity
        double position = this->joint->Position(0);
        double velocity = this->joint->GetVelocity(0);

        // double torque = -this->stiffness * position - this->damping * 0.5 * velocity;
        double torque = -this->stiffness * position - this->damping * velocity;

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
  GZ_REGISTER_MODEL_PLUGIN(SerialElasticPlugin)
}
