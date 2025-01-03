#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "cart_pos/msg/cartposvector.hpp" // Custom message
#include "std_msgs/msg/float32_multi_array.hpp"  // Import the standard Float32 message
#include <array>

void quaternionToTransformationMatrix(double x, double y, double z, double w, double t_x, double t_y, double t_z, std::array<std::array<double, 4>, 4> &R) {
    R[0][0] = 1 - 2 * (y * y + z * z);
    R[0][1] = 2 * (x * y - w * z);
    R[0][2] = 2 * (x * z + w * y);

    R[1][0] = 2 * (x * y + w * z);
    R[1][1] = 1 - 2 * (x * x + z * z);
    R[1][2] = 2 * (y * z - w * x);

    R[2][0] = 2 * (x * z - w * y);
    R[2][1] = 2 * (y * z + w * x);
    R[2][2] = 1 - 2 * (x * x + y * y);

    R[0][3] = t_x;
    R[1][3] = t_y;
    R[2][3] = t_z;

    R[3][3] = 1;
    R[3][0] = 0;
    R[3][0] = 0;
    R[3][0] = 0;

}
using Matrix4x4 = std::array<std::array<double, 4>, 4>;  // 4x4 matrix
// Matrix multiplication
Matrix4x4 multiply_matrices(const Matrix4x4& A, const Matrix4x4& B) {
    Matrix4x4 result = {};
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            for (size_t k = 0; k < 4; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return result;
}

class CartPublisher : public rclcpp::Node {
public:
    CartPublisher() : Node("cart_pos_publisher") {
        subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10,
            std::bind(&CartPublisher::tf_callback, this, std::placeholders::_1)
        );

        // Publisher for the translation-only vector
        publisher_ = this->create_publisher<cart_pos::msg::Cartposvector>(
            "/cart_pos", 10
        );

        RCLCPP_INFO(this->get_logger(), "Cart Pose Publisher Node started.");
    }

private:
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Received %lu transforms from /tf", msg->transforms.size());
      std::array<std::array<double, 4>, 4> endMatrix = {{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
      }};
      int i = 0; 
      for (const auto &transform : msg->transforms) {
        // Create a new Float32 message to publish translation data
        auto message = cart_pos::msg::Cartposvector();

        // You can choose to publish the translation as a single float (e.g., x, y, or z)
        // For example, let's publish the x translation
        double t_x = transform.transform.translation.x;
        double t_y = transform.transform.translation.y;
        double t_z = transform.transform.translation.z;

        double x = transform.transform.rotation.x;
        double y = transform.transform.rotation.y;
        double z = transform.transform.rotation.z;
        double w = transform.transform.rotation.w;

        // Initialize rotation matrix
        std::array<std::array<double, 4>, 4> transMatrix;

        // Compute the rotation matrix
        quaternionToTransformationMatrix(x, y, z, w,t_x,t_y,t_z, transMatrix);
        
        i++; 
        if (i > 1){
            endMatrix = multiply_matrices(endMatrix,transMatrix);
            //endMatrix = transMatrices[1];
        }
        // RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", transform.header.frame_id.c_str());
        // RCLCPP_INFO(this->get_logger(), "  Child Frame ID: %s", transform.child_frame_id.c_str());
        // Output the rotation matrix
        // std::cout << "Trans Matrix:\n";
        // for (const auto &row : transMatrix) {
        //     for (const auto &elem : row) {
        //         std::cout << elem << " ";
        //     }
        //     std::cout << "\n";
        // }

        //float_msg.data = {1,1,1};
        message.x = endMatrix[0][3]; 
        message.y = endMatrix[1][3]; 
        message.z = endMatrix[2][3]; 
        // message.x = 1; 
        // message.y = 2; 
        // message.z = 3; 
        // Publish the translation data (x value in this case)
        publisher_->publish(message);
      }

    //   std::cout << "End Effector Matrix:\n";
    //   for (const auto &row : endMatrix) {
    //     for (const auto &elem : row) {
    //         std::cout << elem << " ";
    //     }
    //     std::cout << "\n";
    //   }
      

    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    rclcpp::Publisher<cart_pos::msg::Cartposvector>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartPublisher>());
    rclcpp::shutdown();
    return 0;
}
