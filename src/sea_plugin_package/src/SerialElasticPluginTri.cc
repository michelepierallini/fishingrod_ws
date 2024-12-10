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


#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>

namespace gazebo
{
  class SerialElasticPluginTri : public ModelPlugin
  {
    public: 
      SerialElasticPluginTri() : ModelPlugin() {}

      virtual ~SerialElasticPluginTri() {}

      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        this->model = _model;
        this->world = _model->GetWorld();

        // Load joint names
        if (_sdf->HasElement("joints"))
        {
          sdf::ElementPtr jointElem = _sdf->GetElement("joints");
          while (jointElem)
          {
            this->jointNames.push_back(jointElem->Get<std::string>());
            jointElem = jointElem->GetNextElement("joint");
          }
        }

        // Parse stiffness and damping matrices from the SDF
        if (_sdf->HasElement("stiffness"))
        {
          std::string stiffnessData = _sdf->GetElement("stiffness")->Get<std::string>();
          this->K_matrix = ParseMatrix(stiffnessData, this->jointNames.size());
        }

        if (_sdf->HasElement("damping"))
        {
          std::string dampingData = _sdf->GetElement("damping")->Get<std::string>();
          this->D_matrix = ParseMatrix(dampingData, this->jointNames.size());
        }

        // Initialize joint forces
        this->torques.resize(this->jointNames.size(), 0.0);

        // Set up a connection to update the forces each timestep
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&SerialElasticPluginTri::OnUpdate, this));
      }

      // Method to parse tridiagonal matrices (K_matrix and D_matrix)
      Eigen::MatrixXd ParseMatrix(const std::string &data, size_t size)
      {
        Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size, size);
        std::istringstream iss(data);
        
        // Fill the diagonal
        for (size_t i = 0; i < size; ++i)
        {
          iss >> matrix(i, i);  // main diagonal
        }
        
        // Fill the sub-diagonal and super-diagonal
        for (size_t i = 1; i < size; ++i)
        {
          double value;
          iss >> value;
          matrix(i, i - 1) = value / 2e1;  // sub-diagonal
          matrix(i - 1, i) = value / 2e1;  // super-diagonal
        }
        return matrix;
      }

      // Method to update joint forces based on stiffness and damping
      void OnUpdate()
      {
        // Get joint positions and velocities
        std::vector<double> positions;
        std::vector<double> velocities;
        for (size_t i = 0; i < this->jointNames.size(); ++i)
        {
          physics::JointPtr joint = this->model->GetJoint(this->jointNames[i]);
          positions.push_back(joint->Position(0));
          velocities.push_back(joint->GetVelocity(0));
        }

        // Calculate torques based on stiffness and damping
        for (size_t jointIndex = 0; jointIndex < this->jointNames.size(); ++jointIndex)
        {
          torques[jointIndex] = -this->K_matrix(jointIndex, jointIndex) * positions[jointIndex]
                                - this->D_matrix(jointIndex, jointIndex) * velocities[jointIndex];
          
          // Apply interaction from neighboring joints (for tridiagonal matrix)
          if (jointIndex > 0)
          {
            torques[jointIndex] -= this->K_matrix(jointIndex, jointIndex - 1) * positions[jointIndex - 1]
                                    - this->D_matrix(jointIndex, jointIndex - 1) * velocities[jointIndex - 1];
          }

          if (jointIndex < this->jointNames.size() - 1)
          {
            torques[jointIndex] -= this->K_matrix(jointIndex, jointIndex + 1) * positions[jointIndex + 1]
                                    - this->D_matrix(jointIndex, jointIndex + 1) * velocities[jointIndex + 1];
          }

          // Apply the computed torque to the joint
          physics::JointPtr joint = this->model->GetJoint(this->jointNames[jointIndex]);
          joint->SetForce(0, torques[jointIndex]);
        }
      }

    private:
      physics::ModelPtr model;
      physics::WorldPtr world;
      event::ConnectionPtr updateConnection;
      
      // Vectors to store joint names and torques
      std::vector<std::string> jointNames;
      std::vector<double> torques;

      // Matrices for stiffness and damping
      Eigen::MatrixXd K_matrix;
      Eigen::MatrixXd D_matrix;
  };

  // Register the plugin with Gazebo
  GZ_REGISTER_MODEL_PLUGIN(SerialElasticPluginTri)
}
