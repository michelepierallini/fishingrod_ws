// #include <gazebo/common/Plugin.hh>
// #include <gazebo/physics/physics.hh>
// #include <ignition/math/Vector3.hh>
// #include <iostream>
// #include <Eigen/Dense>

// namespace gazebo
// {
//   class SerialElasticPluginTri : public ModelPlugin
//   {
//   public:
//     void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
//     {
//       this->model = model;

//       // Parse joint names
//       if (sdf->HasElement("joint_names"))
//       {
//         std::string jointNamesStr = sdf->Get<std::string>("joint_names");
//         std::istringstream iss(jointNamesStr);
//         std::string jointName;
//         while (iss >> jointName)
//         {
//           auto joint = this->model->GetJoint(jointName);
//           if (!joint)
//           {
//             gzerr << "Joint " << jointName << " not found in the model!\n";
//             return;
//           }
//           this->joints.push_back(joint);
//         }
//       }
//       else
//       {
//         gzerr << "No joint_names specified in the SDF plugin element.\n";
//         return;
//       }

//       // Parse K and D matrix values (assume they are diagonal for simplicity)
//       if (sdf->HasElement("stiffness"))
//       {
//         this->K_matrix = ParseMatrix(sdf->Get<std::string>("stiffness"), this->joints.size());
//       }
//       if (sdf->HasElement("damping"))
//       {
//         this->D_matrix = ParseMatrix(sdf->Get<std::string>("damping"), this->joints.size());
//       }

//       // Connect to the world update event
//       this->updateConnection = event::Events::ConnectWorldUpdateBegin(
//           std::bind(&SerialElasticPluginTri::OnUpdate, this));
//     }

//     // void OnUpdate()
//     // {
//     //   Eigen::VectorXd positions(this->joints.size());
//     //   Eigen::VectorXd velocities(this->joints.size());
//     //   Eigen::VectorXd torques(this->joints.size());

//     //   // Get joint positions and velocities
//     //   for (size_t i = 0; i < this->joints.size(); ++i)
//     //   {
//     //     positions[i] = this->joints[i]->Position(0);
//     //     velocities[i] = this->joints[i]->GetVelocity(0);
//     //   }

//     //   // Compute torques using K_matrix and D_matrix
//     //   torques = -this->K_matrix * positions - this->D_matrix * velocities;

//     //   // Apply torques to the joints
//     //   for (size_t i = 0; i < this->joints.size(); ++i)
//     //   {
//     //     this->joints[i]->SetForce(0, torques[i]);
//     //   }
//     // }

//     void OnUpdate()
//     {
//       Eigen::VectorXd positions(this->joints.size());
//       Eigen::VectorXd velocities(this->joints.size());
//       Eigen::VectorXd torques = Eigen::VectorXd::Zero(this->joints.size());

//       // Get joint positions and velocities
//       for (size_t i = 0; i < this->joints.size(); ++i)
//       {
//         positions[i] = this->joints[i]->Position(0);
//         velocities[i] = this->joints[i]->GetVelocity(0);
//       }

//       // Compute torques for specific joint (e.g., joint2)
//       size_t jointIndex = 1; // Assuming `joint2` is the second joint
//       torques[jointIndex] = -this->K_matrix(jointIndex, jointIndex) * positions[jointIndex]
//                             - this->D_matrix(jointIndex, jointIndex) * velocities[jointIndex];

//       // Apply torques to the specific joint
//       this->joints[jointIndex]->SetForce(0, torques[jointIndex]);
//     }

//   private:
//     Eigen::MatrixXd ParseMatrix(const std::string &data, size_t size)
//     {
//       Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size, size);
//       std::istringstream iss(data);
//       for (size_t i = 0; i < size; ++i)
//       {
//         for (size_t j = 0; j < size; ++j)
//         {
//           iss >> matrix(i, j);
//         }
//       }
//       return matrix;
//     }

//     physics::ModelPtr model;
//     std::vector<physics::JointPtr> joints;
//     Eigen::MatrixXd K_matrix;
//     Eigen::MatrixXd D_matrix;
//     event::ConnectionPtr updateConnection;
//   };

//   // Register this plugin with the simulator
//   GZ_REGISTER_MODEL_PLUGIN(SerialElasticPluginTri)
// }

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <vector>
#include <sstream>

/* This class implements a Serial Elastic Actuator (SEA)
   with coupling between neighboring joints.
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
      
      const double scale = 0.0; // Define the constant value

      for (size_t i = 0; i < this->joints.size(); ++i)
      {
        double position = this->joints[i]->Position(0);
        double velocity = this->joints[i]->GetVelocity(0);
        double torque = -this->stiffness[i] * position - this->damping[i] * velocity;

        // Coupling effects with neighboring joints
        if (i > 0) // Previous joint
        {
          double prevPosition = this->joints[i - 1]->Position(0);
          double prevVelocity = this->joints[i - 1]->GetVelocity(0);

          torque += -scale * this->stiffness[i - 1] * prevPosition 
                    - scale * this->damping[i - 1] * prevVelocity;
        }

        if (i < this->joints.size() - 1) // Next joint
        {
          double nextPosition = this->joints[i + 1]->Position(0);
          double nextVelocity = this->joints[i + 1]->GetVelocity(0);

          torque += -scale * this->stiffness[i + 1] * nextPosition 
                    - scale * this->damping[i + 1] * nextVelocity;
        }

        this->joints[i]->SetForce(0, torque);
      }
    }

  private:
    // Helper function to parse a string of double values into a vector
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
