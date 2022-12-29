#include "rclcpp/rclcpp.hpp"
#include "robotic_hw_solution/srv/sensor.hpp"
#include <std_msgs/msg/float64_multi_array.h>
#include <mutex>
#include <memory>

using namespace std::chrono_literals;

class SensorServiceClient : public rclcpp::Node{
    // Class that makes calls to the read_sensor service and publishes the
    // filtered data to a topic for each sensor
    public:
        SensorServiceClient();

    private:
        struct SensorManager{
            // Struct used for handling callbacks, etc. for each sensor 
            // service client
            void update();
            void publish();
            std::string sensor_name_;
            rclcpp::Client<robotic_hw_solution::srv::Sensor>::SharedPtr client_;
            rclcpp::TimerBase::SharedPtr update_timer_;
            rclcpp::TimerBase::SharedPtr publish_timer_;
            rclcpp::callback_group::CallbackGroup::SharedPtr client_cb_group_;
            rclcpp::callback_group::CallbackGroup::SharedPtr update_cb_group_;
            std_msgs::msg::Float64MultiArray data_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
            std::mutex mutex_; // Mutex for locking during access to data_
        };

    std::vector<std::shared_ptr<SensorManager>> sensor_managers_;
};

SensorServiceClient::SensorServiceClient() : Node("sensor_service_client"){
    // Get vector of sensor names. Need to create client for each sensor
    this->declare_parameter("sensor_names", std::vector<std::string>());
    std::vector<std::string> names = this->get_parameter("sensor_names").get_parameter_value().get<std::vector<std::string>>();

    for (auto name : names){
        // For each name in sensor_names, create a sensor_manager, and set up
        // the clients and publishers
        std::shared_ptr<SensorManager> sensor_manager = std::make_shared<SensorManager>();

        sensor_manager->sensor_name_ = name;
        // Use different callback groups so that each service client and update
        // timer thread can run in parallel
        sensor_manager->client_cb_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
        sensor_manager->update_cb_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

        // Create client
        sensor_manager->client_ = this->create_client<robotic_hw_solution::srv::Sensor>(
                                            name + "/read_sensor",
                                            rmw_qos_profile_services_default,
                                            sensor_manager->client_cb_group_);

        // Wait for the service to start. If the service does not start
        // after waiting, skip this sensor and move on
        RCLCPP_INFO(get_logger(), "Waiting for service %s to start...", 
                                sensor_manager->client_->get_service_name());
        if (!sensor_manager->client_->wait_for_service(10s)){
            RCLCPP_ERROR(get_logger(), "Timed out waiting for service %s. "
                                    "Aborting client for sensor %s.",
                                    sensor_manager->client_->get_service_name(),
                                    name.c_str());
            continue;
        }
        RCLCPP_INFO(get_logger(), "Service %s started.", 
                                sensor_manager->client_->get_service_name());

        // Set up publishers and timers
        sensor_manager->publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                                                    name + "/sensor_data", 10);

        sensor_manager->update_timer_ = this->create_wall_timer(0s,
                        std::bind(&SensorServiceClient::SensorManager::update,
                        sensor_manager), sensor_manager->update_cb_group_);
        sensor_manager->publish_timer_ = this->create_wall_timer(2ms,
                        std::bind(&SensorServiceClient::SensorManager::publish,
                        sensor_manager));

        sensor_managers_.push_back(sensor_manager);
    }
}

void SensorServiceClient::SensorManager::update(){
    // Call service
    auto request = std::make_shared<robotic_hw_solution::srv::Sensor::Request>();
    auto result = client_->async_send_request(request);

    // Wait for the result
    auto status = result.wait_for(3s);
    if (status != std::future_status::ready){
        // If timed out, skip
        RCLCPP_ERROR( rclcpp::get_logger("sensor_manager/"+sensor_name_),
                                            "Failed to read sensor. Skipping.");
        return;
    }

    // Store result in data_
    std::unique_lock<std::mutex> lock(mutex_);
    data_ = result.get()->data;
    lock.unlock();
}

void SensorServiceClient::SensorManager::publish(){
    // Publish data
    std::unique_lock<std::mutex> lock(mutex_);
    publisher_->publish(data_);
    lock.unlock();
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    // Use MultiThreadedExecutor so that threads can run in parallel
    rclcpp::executors::MultiThreadedExecutor exec;
    std::shared_ptr<SensorServiceClient> node = std::make_shared<SensorServiceClient>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}