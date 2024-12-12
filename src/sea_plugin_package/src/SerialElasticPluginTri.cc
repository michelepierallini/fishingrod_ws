#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <vector>
#include <sstream>

/* This class implements a Serial Elastic Actuator (SEA)
  with coupling between neighboring joints.
  The issue is within the damping which is too high for Gazebo, CHANGE IT AND RETRY 
*/

namespace gazebo
{
  class SerialElasticPluginTri : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      this->model = model;

      // Parse stiffness and damping values
      if (sdf->HasElement("stiffness"))
      {
        std::string stiffnessStr = sdf->Get<std::string>("stiffness");
        this->stiffness = ParseValues(stiffnessStr);
      }
      else
      {
        gzerr << "Missing <stiffness> parameter in SDF plugin element.\n";
        return;
      }

      if (sdf->HasElement("damping"))
      {
        std::string dampingStr = sdf->Get<std::string>("damping");
        this->damping = ParseValues(dampingStr);
      }
      else
      {
        gzerr << "Missing <damping> parameter in SDF plugin element.\n";
        return;
      }

      // Get joint names
      if (sdf->HasElement("joint_name"))
      {
        std::string jointNamesStr = sdf->Get<std::string>("joint_name");
        std::istringstream iss(jointNamesStr);
        std::string jointName;
        while (iss >> jointName)
        {
          auto joint = this->model->GetJoint(jointName);
          if (joint)
            this->joints.push_back(joint);
          else
            gzerr << "Joint " << jointName << " not found in the model!\n";
        }
      }
      else
      {
        gzerr << "No <joint_name> specified in the SDF plugin element.\n";
        return;
      }

      if (this->joints.size() != this->stiffness.size() || 
          this->joints.size() != this->damping.size())
      {
        gzerr << "Mismatch between number of joints, stiffness, and damping values.\n";
        return;
      }

      // Connect to the world update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SerialElasticPluginTri::OnUpdate, this));
    }

    void OnUpdate()
    {
      if (this->joints.empty())
        return;
      
      double scale = 0.5;
      double scale_damp = 1e-1;

      for (size_t i = 0; i < this->joints.size(); ++i)
      {
        double position = this->joints[i]->Position(0);
        double velocity = this->joints[i]->GetVelocity(0);
        double torque = -this->stiffness[i] * position - this->damping[i] * scale_damp * velocity;

        if (i > 0) // Previous
        {
          double prevPosition = this->joints[i - 1]->Position(0);
          double prevVelocity = this->joints[i - 1]->GetVelocity(0);

          torque += -scale * this->stiffness[i - 1] * prevPosition / 2
                    - scale_damp * this->damping[i - 1] * prevVelocity / 2;
        }

        if (i < this->joints.size() - 1) // Next
        {
          double nextPosition = this->joints[i + 1]->Position(0);
          double nextVelocity = this->joints[i + 1]->GetVelocity(0);

          torque += -scale * this->stiffness[i + 1] * nextPosition / 2
                    - scale_damp * this->damping[i + 1] * nextVelocity / 2;
        }
        this->joints[i]->SetForce(0, torque);
      }
    }

  private:
    std::vector<double> ParseValues(const std::string &data)
    {
      std::vector<double> values;
      std::istringstream iss(data);
      double value;
      while (iss >> value)
      {
        values.push_back(value);
      }
      return values;
    }

    physics::ModelPtr model;
    std::vector<physics::JointPtr> joints;
    std::vector<double> stiffness; // Stiffness values for each joint
    std::vector<double> damping;   // Damping values for each joint
    event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(SerialElasticPluginTri)
}
