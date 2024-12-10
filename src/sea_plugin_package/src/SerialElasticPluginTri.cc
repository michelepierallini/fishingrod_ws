#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <Eigen/Dense>

namespace gazebo
{
  class SerialElasticPluginTri : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      this->model = model;

      // Parse joint names
      if (sdf->HasElement("joint_names"))
      {
        std::string jointNamesStr = sdf->Get<std::string>("joint_names");
        std::istringstream iss(jointNamesStr);
        std::string jointName;
        while (iss >> jointName)
        {
          auto joint = this->model->GetJoint(jointName);
          if (!joint)
          {
            gzerr << "Joint " << jointName << " not found in the model!\n";
            return;
          }
          this->joints.push_back(joint);
        }
      }
      else
      {
        gzerr << "No joint_names specified in the SDF plugin element.\n";
        return;
      }

      // Parse K and D matrix values (assume they are diagonal for simplicity)
      if (sdf->HasElement("stiffness"))
      {
        this->K_matrix = ParseMatrix(sdf->Get<std::string>("stiffness"), this->joints.size());
      }
      if (sdf->HasElement("damping"))
      {
        this->D_matrix = ParseMatrix(sdf->Get<std::string>("damping"), this->joints.size());
      }

      // Connect to the world update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SerialElasticPluginTri::OnUpdate, this));
    }

    // void OnUpdate()
    // {
    //   Eigen::VectorXd positions(this->joints.size());
    //   Eigen::VectorXd velocities(this->joints.size());
    //   Eigen::VectorXd torques(this->joints.size());

    //   // Get joint positions and velocities
    //   for (size_t i = 0; i < this->joints.size(); ++i)
    //   {
    //     positions[i] = this->joints[i]->Position(0);
    //     velocities[i] = this->joints[i]->GetVelocity(0);
    //   }

    //   // Compute torques using K_matrix and D_matrix
    //   torques = -this->K_matrix * positions - this->D_matrix * velocities;

    //   // Apply torques to the joints
    //   for (size_t i = 0; i < this->joints.size(); ++i)
    //   {
    //     this->joints[i]->SetForce(0, torques[i]);
    //   }
    // }

    void OnUpdate()
    {
      Eigen::VectorXd positions(this->joints.size());
      Eigen::VectorXd velocities(this->joints.size());
      Eigen::VectorXd torques = Eigen::VectorXd::Zero(this->joints.size());

      // Get joint positions and velocities
      for (size_t i = 0; i < this->joints.size(); ++i)
      {
        positions[i] = this->joints[i]->Position(0);
        velocities[i] = this->joints[i]->GetVelocity(0);
      }

      // Compute torques for specific joint (e.g., joint2)
      size_t jointIndex = 1; // Assuming `joint2` is the second joint
      torques[jointIndex] = -this->K_matrix(jointIndex, jointIndex) * positions[jointIndex]
                            - this->D_matrix(jointIndex, jointIndex) * velocities[jointIndex];

      // Apply torques to the specific joint
      this->joints[jointIndex]->SetForce(0, torques[jointIndex]);
    }

  private:
    Eigen::MatrixXd ParseMatrix(const std::string &data, size_t size)
    {
      Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size, size);
      std::istringstream iss(data);
      for (size_t i = 0; i < size; ++i)
      {
        for (size_t j = 0; j < size; ++j)
        {
          iss >> matrix(i, j);
        }
      }
      return matrix;
    }

    physics::ModelPtr model;
    std::vector<physics::JointPtr> joints;
    Eigen::MatrixXd K_matrix;
    Eigen::MatrixXd D_matrix;
    event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SerialElasticPluginTri)
}
