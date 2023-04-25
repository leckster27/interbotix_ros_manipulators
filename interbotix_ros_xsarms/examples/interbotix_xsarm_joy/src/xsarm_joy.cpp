// Copyright 2022 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "interbotix_xs_msgs/msg/arm_joy.hpp"

typedef std::map<std::string, int> button_mappings;

// PS3 Controller button mappings
static const button_mappings ps3 = {
  {"GRIPPER_PWM_DEC", 0},  // buttons start here
  {"GRIPPER_RELEASE", 1},
  {"GRIPPER_PWM_INC", 2},
  {"GRIPPER_GRASP", 3},
  {"EE_Y_INC", 4},
  {"EE_Y_DEC", 5},
  {"WAIST_CCW", 6},
  {"WAIST_CW", 7},
  {"SLEEP_POSE", 8},
  {"HOME_POSE", 9},
  {"TORQUE_ENABLE", 10},
  {"FLIP_EE_X", 11},
  {"FLIP_EE_ROLL", 12},
  {"SPEED_INC", 13},
  {"SPEED_DEC", 14},
  {"SPEED_COARSE", 15},
  {"SPEED_FINE", 16},
  {"EE_X", 0},             // axes start here
  {"EE_Z", 1},
  {"EE_ROLL", 3},
  {"EE_PITCH", 4}
};

// PS4 Controller button mappings
static const button_mappings ps4 = {
  {"GRIPPER_PWM_DEC", 0},  // buttons start here
  {"GRIPPER_RELEASE", 1},
  {"GRIPPER_PWM_INC", 2},
  {"GRIPPER_GRASP", 3},
  {"EE_Y_INC", 4},
  {"EE_Y_DEC", 5},
  {"WAIST_CCW", 6},
  {"WAIST_CW", 7},
  {"SLEEP_POSE", 8},
  {"HOME_POSE", 9},
  {"TORQUE_ENABLE", 10},
  {"FLIP_EE_X", 11},
  {"FLIP_EE_ROLL", 12},
  {"EE_X", 0},             // axes start here
  {"EE_Z", 1},
  {"EE_ROLL", 3},
  {"EE_PITCH", 4},
  {"SPEED_TYPE", 6},
  {"SPEED", 7}
};

// Xbox 360 Controller button mappings
static const button_mappings xbox360 = {
  {"GRIPPER_PWM_DEC", 0},  // buttons start here
  {"GRIPPER_RELEASE", 1},
  {"GRIPPER_GRASP", 2},
  {"GRIPPER_PWM_INC", 3},
  {"WAIST_CCW", 4},
  {"WAIST_CW", 5},
  {"SLEEP_POSE", 6},
  {"HOME_POSE", 7},
  {"TORQUE_ENABLE", 8},
  {"FLIP_EE_X", 9},
  {"FLIP_EE_ROLL", 10},
  {"EE_X", 0},             // axes start here
  {"EE_Z", 1},
  {"EE_Y_INC", 2},
  {"EE_ROLL", 3},
  {"EE_PITCH", 4},
  {"EE_Y_DEC", 5},
  {"SPEED_TYPE", 6},
  {"SPEED", 7}
};

// USB Joystick button mappings
static const button_mappings usbjoy = {
  // buttons start here
  {"HOME_POSE", 0}, //BLUE
  {"GRIPPER_GRASP", 1}, //GREEN
  {"GRIPPER_RELEASE", 2}, //YELLOW
  {"SLEEP_POSE", 3}, //RED
  {"EE_Z_INC", 4}, //BLACK1 Z-Increase button
  {"EE_Z_DEC", 5}, //BLACK2 Z-Decrease button
  {"TOGGLE_LOGGING", 6}, //BLACK3 - unmapped currntly just used for turning on debug logging
  // axes start here
  {"EE_WAIST", 0}, // Left/Right Axis. Controls WAIST as an axis (turn side to side)
  {"EE_X", 1} // Forward/Back Axis. Controls arm in and out.
};

const std::string controller_type_ps3 = "ps3";
const std::string controller_type_ps4 = "ps4";
const std::string controller_type_xbox360 = "xbox360";
const std::string controller_type_usbjoy = "usbjoy";

class InterbotixXSArmJoy : public rclcpp::Node
{
public:
  // ROS Subscription to receive raw Joy messages
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_raw;

  // ROS Publisher to publish ArmJoy messages
  rclcpp::Publisher<interbotix_xs_msgs::msg::ArmJoy>::SharedPtr pub_joy_cmd;

  // Keep track of the previously commanded ArmJoy message so that only unique messages are
  // published
  interbotix_xs_msgs::msg::ArmJoy prev_joy_cmd;

  // Holds the controller button mappings
  button_mappings cntlr = ps4;

  bool enableLogging = true;

  // Holds the name of the controller received from the ROS Parameter server
  std::string controller_type;

  // Joystick sensitivity threshold
  double threshold;

  explicit InterbotixXSArmJoy(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("xsarm_joy", options)
  {
    this->declare_parameter("threshold", 0.75);
    this->declare_parameter("controller", controller_type_ps4);

    this->get_parameter("threshold", threshold);
    this->get_parameter("controller", controller_type);

    if (controller_type == controller_type_xbox360) {
      cntlr = xbox360;
    } else if (controller_type == controller_type_ps3) {
      cntlr = ps3;
    } else if (controller_type == controller_type_ps4) {
      cntlr = ps4;
    } else if (controller_type == controller_type_usbjoy) {
      cntlr = usbjoy;
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Unsupported controller: '%s'. Defaulting to ps4 button mappings.",
        controller_type.c_str());
    }

    pub_joy_cmd = this->create_publisher<interbotix_xs_msgs::msg::ArmJoy>(
      "commands/joy_processed",
      10);
    sub_joy_raw = this->create_subscription<sensor_msgs::msg::Joy>(
      "commands/joy_raw",
      10,
      [this](sensor_msgs::msg::Joy msg) {joy_state_cb(msg);});
  }

private:
  /// @brief Joystick callback to create custom ArmJoy messages to control the Arm
  /// @param msg - raw sensor_msgs::msg::Joy data
  void joy_state_cb(const sensor_msgs::msg::Joy & msg)
  {
    static bool flip_ee_roll_cmd = false;
    static bool flip_ee_roll_cmd_last_state = false;
    static bool flip_ee_x_cmd = false;
    static bool flip_ee_x_cmd_last_state = false;
    static bool flip_torque_cmd = true;
    static bool flip_torque_cmd_last_state = true;
    static double time_start;
    static bool timer_started = false;
    interbotix_xs_msgs::msg::ArmJoy joy_cmd;

    std::string buttonsMessage = "";
    std::string axesMessage = "";

    if (enableLogging)
    {
      bool shouldLog = false;
      for (int i = 0; i < 6; i++)
      {
        shouldLog = shouldLog || msg.axes[i] != 0;
        axesMessage += std::to_string(msg.axes[i])+',';
      }
      for (int i = 0; i < 12; i++)
      {
        shouldLog = shouldLog || msg.buttons[i] != 0;
        buttonsMessage += std::to_string(msg.buttons[i])+',';
      }

      if (shouldLog)
      {
        RCLCPP_WARN(
            this->get_logger(),
            "Axes input: '%s'.",
            axesMessage.c_str());
        RCLCPP_WARN(
            this->get_logger(),
            "Buttons input: '%s'.",
            buttonsMessage.c_str());
      }
    }

    // Check the toggle logging input
    if (msg.buttons.at(cntlr["TOGGLE_LOGGING"]) == 1)
    {
      //enableLogging = !enableLogging;
    }

    // Check if the torque_cmd should be flipped
    if (msg.buttons.at(cntlr["TORQUE_ENABLE"]) == 1 && !flip_torque_cmd_last_state) {
      flip_torque_cmd = true;
      joy_cmd.torque_cmd = interbotix_xs_msgs::msg::ArmJoy::TORQUE_ON;
      RCLCPP_WARN( this->get_logger(), "Setting torque_cmd to TORQUE_ON because flip_torque_cmd_last_state is false and buttons.at(cntlr['TORQUE_ENABLE']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["TORQUE_ENABLE"]).c_str(), std::to_string(msg.buttons.at(cntlr["TORQUE_ENABLE"])).c_str());
    } else if (msg.buttons.at(cntlr["TORQUE_ENABLE"]) == 1 && flip_torque_cmd_last_state) {
      time_start = this->get_clock()->now().seconds();
      timer_started = true;
    } else if (msg.buttons.at(cntlr["TORQUE_ENABLE"]) == 0) {
      if (timer_started && this->get_clock()->now().seconds() - time_start > 3.0) {
        joy_cmd.torque_cmd = interbotix_xs_msgs::msg::ArmJoy::TORQUE_OFF;
        RCLCPP_WARN( this->get_logger(), "Setting torque_cmd to TORQUE_OFF because timer is > 3.0 and buttons.at(cntlr['TORQUE_ENABLE']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["TORQUE_ENABLE"]).c_str(), std::to_string(msg.buttons.at(cntlr["TORQUE_ENABLE"])).c_str());
        flip_torque_cmd = false;
      }
      flip_torque_cmd_last_state = flip_torque_cmd;
      timer_started = false;
    }

    // Check if the ee_x_cmd should be flipped
    if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 1 && !flip_ee_x_cmd_last_state) {
      flip_ee_x_cmd = true;
      RCLCPP_WARN( this->get_logger(), "Setting flip_ee_x_cmd to true because flip_ee_x_cmd_last_state is false and buttons.at(cntlr['FLIP_EE_X']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["FLIP_EE_X"]).c_str(), std::to_string(msg.buttons.at(cntlr["FLIP_EE_X"])).c_str());
    } else if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 1 && flip_ee_x_cmd_last_state) {
      flip_ee_x_cmd = false;
      RCLCPP_WARN( this->get_logger(), "Setting flip_ee_x_cmd to false because flip_ee_x_cmd_last_state is true and buttons.at(cntlr['FLIP_EE_X']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["FLIP_EE_X"]).c_str(), std::to_string(msg.buttons.at(cntlr["FLIP_EE_X"])).c_str());
    } else if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 0) {
      flip_ee_x_cmd_last_state = flip_ee_x_cmd;
    }

    // Check the ee_x_cmd
    if (msg.axes.at(cntlr["EE_X"]) >= threshold && !flip_ee_x_cmd) {
      joy_cmd.ee_x_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_X_INC;
      RCLCPP_WARN( this->get_logger(), "Setting ee_x_cmd to EE_X_INC because flip_ee_x_cmd is false and axes.at(cntlr['EE_X']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_X"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_X"])).c_str());
    } else if (msg.axes.at(cntlr["EE_X"]) <= -threshold && !flip_ee_x_cmd) {
      joy_cmd.ee_x_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_X_DEC;
      RCLCPP_WARN( this->get_logger(), "Setting ee_x_cmd to EE_X_DEC because flip_ee_x_cmd is false and axes.at(cntlr['EE_X']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_X"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_X"])).c_str());
    } else if (msg.axes.at(cntlr["EE_X"]) >= threshold && flip_ee_x_cmd) {
      joy_cmd.ee_x_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_X_DEC;
      RCLCPP_WARN( this->get_logger(), "Setting ee_x_cmd to EE_X_DEC because flip_ee_x_cmd is true and axes.at(cntlr['EE_X']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_X"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_X"])).c_str());
    } else if (msg.axes.at(cntlr["EE_X"]) <= -threshold && flip_ee_x_cmd) {
      joy_cmd.ee_x_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_X_INC;
      RCLCPP_WARN( this->get_logger(), "Setting ee_x_cmd to EE_X_INC because flip_ee_x_cmd is true and  axes.at(cntlr['EE_X']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_X"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_X"])).c_str());
    }

    // Check the ee_y_cmd
    if (controller_type == controller_type_ps3 || controller_type == controller_type_ps4) {
      if (msg.buttons.at(cntlr["EE_Y_INC"]) == 1) {
        joy_cmd.ee_y_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Y_INC;
        RCLCPP_WARN( this->get_logger(), "Setting ee_y_cmd to EE_Y_INC because ps3/ps4 controller type is used and buttons.at(cntlr['EE_Y_INC']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_Y_INC"]).c_str(), std::to_string(msg.buttons.at(cntlr["EE_Y_INC"])).c_str());
      } else if (msg.buttons.at(cntlr["EE_Y_DEC"]) == 1) {
        joy_cmd.ee_y_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Y_DEC;
        RCLCPP_WARN( this->get_logger(), "Setting ee_y_cmd to EE_Y_DEC because ps3/ps4 controller type is used and buttons.at(cntlr['EE_Y_DEC']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_Y_DEC"]).c_str(), std::to_string(msg.buttons.at(cntlr["EE_Y_DEC"])).c_str());
      }
    } else if (controller_type == controller_type_xbox360) {
      if (msg.axes.at(cntlr["EE_Y_INC"]) <= 1.0 - 2.0 * threshold) {
        joy_cmd.ee_y_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Y_INC;
        RCLCPP_WARN( this->get_logger(), "Setting ee_y_cmd to EE_Y_INC because xbox360 controller type is used and axes.at(cntlr['EE_Y_INC']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_Y_INC"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_Y_INC"])).c_str());
      } else if (msg.axes.at(cntlr["EE_Y_DEC"]) <= 1.0 - 2.0 * threshold) {
        joy_cmd.ee_y_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Y_DEC;
        RCLCPP_WARN( this->get_logger(), "Setting ee_y_cmd to EE_Y_DEC because xbox360 controller type is used and axes.at(cntlr['EE_Y_DEC']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_Y_DEC"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_Y_DEC"])).c_str());
      }
    }

    // Check the ee_z_cmd
    if (controller_type == controller_type_usbjoy)
    {
      if (msg.buttons.at(cntlr["EE_Z_INC"]) == 1) {
        joy_cmd.ee_z_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Z_INC;
        RCLCPP_WARN( this->get_logger(), "Setting ee_z_cmd to EE_Z_INC because usbjoy controller type is used and buttons.at(cntlr['EE_Z_INC']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_Z_INC"]).c_str(), std::to_string(msg.buttons.at(cntlr["EE_Z_INC"])).c_str());
      } else if (msg.buttons.at(cntlr["EE_Z_DEC"]) == 1) {
        joy_cmd.ee_z_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Z_DEC;
        RCLCPP_WARN( this->get_logger(), "Setting ee_z_cmd to EE_Z_DEC because usbjoy controller type is used and buttons.at(cntlr['EE_Z_DEC']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_Z_DEC"]).c_str(), std::to_string(msg.buttons.at(cntlr["EE_Z_DEC"])).c_str());
      }
    }
    else
    {
      if (msg.axes.at(cntlr["EE_Z"]) >= threshold) {
        joy_cmd.ee_z_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Z_INC;
        RCLCPP_WARN( this->get_logger(), "Setting ee_z_cmd to EE_Z_INC because other controller type is used and axes.at(cntlr['EE_Z']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_Z"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_Z"])).c_str());
      } else if (msg.axes.at(cntlr["EE_Z"]) <= -threshold) {
        joy_cmd.ee_z_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_Z_DEC;
        RCLCPP_WARN( this->get_logger(), "Setting ee_z_cmd to EE_Z_DEC because other controller type is used and axes.at(cntlr['EE_Z']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_Z"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_Z"])).c_str());
      }
    }

    // Check if the ee_roll_cmd should be flipped
    if (msg.buttons.at(cntlr["FLIP_EE_ROLL"]) == 1 && !flip_ee_roll_cmd_last_state) {
      flip_ee_roll_cmd = true;
      RCLCPP_WARN( this->get_logger(), "Setting flip_ee_roll_cmd to true because flip_ee_roll_cmd_last_state is false and buttons.at(cntlr['FLIP_EE_ROLL']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["FLIP_EE_ROLL"]).c_str(), std::to_string(msg.buttons.at(cntlr["FLIP_EE_ROLL"])).c_str());
    } else if (msg.buttons.at(cntlr["FLIP_EE_ROLL"]) == 1 && flip_ee_roll_cmd_last_state) {
      flip_ee_roll_cmd = false;
      RCLCPP_WARN( this->get_logger(), "Setting flip_ee_roll_cmd to false because flip_ee_roll_cmd_last_state is true and buttons.at(cntlr['FLIP_EE_ROLL']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["FLIP_EE_ROLL"]).c_str(), std::to_string(msg.buttons.at(cntlr["FLIP_EE_ROLL"])).c_str());
    } else if (msg.buttons.at(cntlr["FLIP_EE_ROLL"]) == 0) {
      flip_ee_roll_cmd_last_state = flip_ee_roll_cmd;
    }

    // Check the ee_roll_cmd
    if (msg.axes.at(cntlr["EE_ROLL"]) >= threshold && !flip_ee_roll_cmd) {
      joy_cmd.ee_roll_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_ROLL_CW;
      RCLCPP_WARN( this->get_logger(), "Setting ee_roll_cmd to EE_ROLL_CW because flip_ee_roll_cmd is false and axes.at(cntlr['EE_ROLL']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_ROLL"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_ROLL"])).c_str());
    } else if (msg.axes.at(cntlr["EE_ROLL"]) <= -threshold && !flip_ee_roll_cmd) {
      joy_cmd.ee_roll_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_ROLL_CCW;
      RCLCPP_WARN( this->get_logger(), "Setting ee_roll_cmd to EE_ROLL_CCW because flip_ee_roll_cmd is false and axes.at(cntlr['EE_ROLL']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_ROLL"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_ROLL"])).c_str());
    } else if (msg.axes.at(cntlr["EE_ROLL"]) >= threshold && flip_ee_roll_cmd) {
      joy_cmd.ee_roll_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_ROLL_CCW;
      RCLCPP_WARN( this->get_logger(), "Setting ee_roll_cmd to EE_ROLL_CCW because flip_ee_roll_cmd is true and axes.at(cntlr['EE_ROLL']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_ROLL"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_ROLL"])).c_str());
    } else if (msg.axes.at(cntlr["EE_ROLL"]) <= -threshold && flip_ee_roll_cmd) {
      joy_cmd.ee_roll_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_ROLL_CW;
      RCLCPP_WARN( this->get_logger(), "Setting ee_roll_cmd to EE_ROLL_CW because flip_ee_roll_cmd is true and axes.at(cntlr['EE_ROLL']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_ROLL"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_ROLL"])).c_str());
    }

    // Check the ee_pitch_cmd
    if (msg.axes.at(cntlr["EE_PITCH"]) >= threshold) {
      joy_cmd.ee_pitch_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_PITCH_UP;
      RCLCPP_WARN( this->get_logger(), "Setting ee_pitch_cmd to EE_PITCH_UP because axes.at(cntlr['EE_PITCH']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_PITCH"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_PITCH"])).c_str());
    } else if (msg.axes.at(cntlr["EE_PITCH"]) <= -threshold) {
      joy_cmd.ee_pitch_cmd = interbotix_xs_msgs::msg::ArmJoy::EE_PITCH_DOWN;
      RCLCPP_WARN( this->get_logger(), "Setting ee_pitch_cmd to EE_PITCH_DOWN because axes.at(cntlr['EE_PITCH']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_PITCH"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_PITCH"])).c_str());
    }

    // Check the waist_cmd
    if (controller_type == controller_type_usbjoy)
    {
      if (msg.axes.at(cntlr["EE_WAIST"]) >= threshold) {
        joy_cmd.waist_cmd = interbotix_xs_msgs::msg::ArmJoy::WAIST_CCW;
        RCLCPP_WARN( this->get_logger(), "Setting waist_cmd to WAIST_CCW because usbjoy controller type is used and axes.at(cntlr['EE_WAIST']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_WAIST"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_WAIST"])).c_str());
      } else if (msg.axes.at(cntlr["EE_WAIST"]) <= -threshold) {
        joy_cmd.waist_cmd = interbotix_xs_msgs::msg::ArmJoy::WAIST_CW;
        RCLCPP_WARN( this->get_logger(), "Setting waist_cmd to WAIST_CW because usbjoy controller type is used and axes.at(cntlr['EE_WAIST']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["EE_WAIST"]).c_str(), std::to_string(msg.axes.at(cntlr["EE_WAIST"])).c_str());
      }
    }
    else
    {
      if (msg.buttons.at(cntlr["WAIST_CCW"]) == 1) {
        joy_cmd.waist_cmd = interbotix_xs_msgs::msg::ArmJoy::WAIST_CCW;
        RCLCPP_WARN( this->get_logger(), "Setting waist_cmd to WAIST_CCW because buttons.at(cntlr['WAIST_CCW']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["WAIST_CCW"]).c_str(), std::to_string(msg.buttons.at(cntlr["WAIST_CCW"])).c_str());
      } else if (msg.buttons.at(cntlr["WAIST_CW"]) == 1) {
        joy_cmd.waist_cmd = interbotix_xs_msgs::msg::ArmJoy::WAIST_CW;
        RCLCPP_WARN( this->get_logger(), "Setting waist_cmd to WAIST_CW because buttons.at(cntlr['WAIST_CW']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["WAIST_CW"]).c_str(), std::to_string(msg.buttons.at(cntlr["WAIST_CW"])).c_str());
      }
    }

    // Check the gripper_cmd
    if (msg.buttons.at(cntlr["GRIPPER_GRASP"]) == 1) {
      joy_cmd.gripper_cmd = interbotix_xs_msgs::msg::ArmJoy::GRIPPER_GRASP;
        RCLCPP_WARN( this->get_logger(), "Setting gripper_cmd to GRIPPER_GRASP because buttons.at(cntlr['GRIPPER_GRASP']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["GRIPPER_GRASP"]).c_str(), std::to_string(msg.buttons.at(cntlr["GRIPPER_GRASP"])).c_str());
    } else if (msg.buttons.at(cntlr["GRIPPER_RELEASE"]) == 1) {
      joy_cmd.gripper_cmd = interbotix_xs_msgs::msg::ArmJoy::GRIPPER_RELEASE;
        RCLCPP_WARN( this->get_logger(), "Setting gripper_cmd to GRIPPER_RELEASE because buttons.at(cntlr['GRIPPER_RELEASE']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["GRIPPER_RELEASE"]).c_str(), std::to_string(msg.buttons.at(cntlr["GRIPPER_RELEASE"])).c_str());
    }

    // Check the pose_cmd
    if (msg.buttons.at(cntlr["HOME_POSE"]) == 1) {
      joy_cmd.pose_cmd = interbotix_xs_msgs::msg::ArmJoy::HOME_POSE;
        RCLCPP_WARN( this->get_logger(), "Setting pose_cmd to HOME_POSE because buttons.at(cntlr['HOME_POSE']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["HOME_POSE"]).c_str(), std::to_string(msg.buttons.at(cntlr["HOME_POSE"])).c_str());
    } else if (msg.buttons.at(cntlr["SLEEP_POSE"]) == 1) {
      joy_cmd.pose_cmd = interbotix_xs_msgs::msg::ArmJoy::SLEEP_POSE;
        RCLCPP_WARN( this->get_logger(), "Setting pose_cmd to SLEEP_POSE because buttons.at(cntlr['SLEEP_POSE']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["SLEEP_POSE"]).c_str(), std::to_string(msg.buttons.at(cntlr["SLEEP_POSE"])).c_str());
    }

    if (controller_type == "ps3") {
      // Check the speed_cmd
      if (msg.buttons.at(cntlr["SPEED_INC"]) == 1) {
        joy_cmd.speed_cmd = interbotix_xs_msgs::msg::ArmJoy::SPEED_INC;
        RCLCPP_WARN( this->get_logger(), "Setting speed_cmd to SPEED_INC because ps3 controller type is used and buttons.at(cntlr['SPEED_INC']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["SPEED_INC"]).c_str(), std::to_string(msg.buttons.at(cntlr["SPEED_INC"])).c_str());
      } else if (msg.buttons.at(cntlr["SPEED_DEC"]) == 1) {
        joy_cmd.speed_cmd = interbotix_xs_msgs::msg::ArmJoy::SPEED_DEC;
        RCLCPP_WARN( this->get_logger(), "Setting speed_cmd to SPEED_DEC because ps3 controller type is used and buttons.at(cntlr['SPEED_DEC']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["SPEED_DEC"]).c_str(), std::to_string(msg.buttons.at(cntlr["SPEED_DEC"])).c_str());
      }

      // Check the speed_toggle_cmd
      if (msg.buttons.at(cntlr["SPEED_COARSE"]) == 1) {
        joy_cmd.speed_toggle_cmd = interbotix_xs_msgs::msg::ArmJoy::SPEED_COARSE;
        RCLCPP_WARN( this->get_logger(), "Setting speed_toggle_cmd to SPEED_COARSE because ps3 controller type is used and buttons.at(cntlr['SPEED_COARSE']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["SPEED_COARSE"]).c_str(), std::to_string(msg.buttons.at(cntlr["SPEED_COARSE"])).c_str());
      } else if (msg.buttons.at(cntlr["SPEED_FINE"]) == 1) {
        joy_cmd.speed_toggle_cmd = interbotix_xs_msgs::msg::ArmJoy::SPEED_FINE;
        RCLCPP_WARN( this->get_logger(), "Setting speed_toggle_cmd to SPEED_FINE because ps3 controller type is used and buttons.at(cntlr['SPEED_FINE']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["SPEED_FINE"]).c_str(), std::to_string(msg.buttons.at(cntlr["SPEED_FINE"])).c_str());
      }
    } else if (controller_type == controller_type_ps4 || controller_type == controller_type_xbox360) {
      // Check the speed_cmd
      if (msg.axes.at(cntlr["SPEED"]) == 1) {
        joy_cmd.speed_cmd = interbotix_xs_msgs::msg::ArmJoy::SPEED_INC;
        RCLCPP_WARN( this->get_logger(), "Setting speed_cmd to SPEED_INC because ps4/xbox360 controller type is used and axes.at(cntlr['SPEED']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["SPEED"]).c_str(), std::to_string(msg.axes.at(cntlr["SPEED"])).c_str());
      } else if (msg.axes.at(cntlr["SPEED"]) == -1) {
        joy_cmd.speed_cmd = interbotix_xs_msgs::msg::ArmJoy::SPEED_DEC;
        RCLCPP_WARN( this->get_logger(), "Setting speed_cmd to SPEED_DEC because ps4/xbox360 controller type is used and axes.at(cntlr['SPEED']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["SPEED"]).c_str(), std::to_string(msg.axes.at(cntlr["SPEED"])).c_str());
      }

      // Check the speed_toggle_cmd
      if (msg.axes.at(cntlr["SPEED_TYPE"]) == 1) {
        joy_cmd.speed_toggle_cmd = interbotix_xs_msgs::msg::ArmJoy::SPEED_COARSE;
        RCLCPP_WARN( this->get_logger(), "Setting speed_toggle_cmd to SPEED_COARSE because ps4/xbox360 controller type is used and axes.at(cntlr['SPEED_TYPE']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["SPEED_TYPE"]).c_str(), std::to_string(msg.axes.at(cntlr["SPEED_TYPE"])).c_str());
      } else if (msg.axes.at(cntlr["SPEED_TYPE"]) == -1) {
        joy_cmd.speed_toggle_cmd = interbotix_xs_msgs::msg::ArmJoy::SPEED_FINE;
        RCLCPP_WARN( this->get_logger(), "Setting speed_toggle_cmd to SPEED_FINE because ps4/xbox360 controller type is used and axes.at(cntlr['SPEED_TYPE']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["SPEED_TYPE"]).c_str(), std::to_string(msg.axes.at(cntlr["SPEED_TYPE"])).c_str());
      }
    }

    // Check the gripper_pwm_cmd
    if (msg.buttons.at(cntlr["GRIPPER_PWM_INC"]) == 1) {
      joy_cmd.gripper_pwm_cmd = interbotix_xs_msgs::msg::ArmJoy::GRIPPER_PWM_INC;
        RCLCPP_WARN( this->get_logger(), "Setting gripper_pwm_cmd to GRIPPER_PWM_INC because buttons.at(cntlr['GRIPPER_PWM_INC']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["GRIPPER_PWM_INC"]).c_str(), std::to_string(msg.buttons.at(cntlr["GRIPPER_PWM_INC"])).c_str());
    } else if (msg.buttons.at(cntlr["GRIPPER_PWM_DEC"]) == 1) {
      joy_cmd.gripper_pwm_cmd = interbotix_xs_msgs::msg::ArmJoy::GRIPPER_PWM_DEC;
        RCLCPP_WARN( this->get_logger(), "Setting gripper_pwm_cmd to GRIPPER_PWM_DEC because buttons.at(cntlr['GRIPPER_PWM_DEC']) was mapped as '%s' and set to '%s'.", std::to_string(cntlr["GRIPPER_PWM_DEC"]).c_str(), std::to_string(msg.buttons.at(cntlr["GRIPPER_PWM_DEC"])).c_str());
    }

    // Only publish a ArmJoy message if any of the following fields have changed.
    if (
      !((prev_joy_cmd.ee_x_cmd == joy_cmd.ee_x_cmd) &&
      (prev_joy_cmd.ee_y_cmd == joy_cmd.ee_y_cmd) &&
      (prev_joy_cmd.ee_z_cmd == joy_cmd.ee_z_cmd) &&
      (prev_joy_cmd.ee_roll_cmd == joy_cmd.ee_roll_cmd) &&
      (prev_joy_cmd.ee_pitch_cmd == joy_cmd.ee_pitch_cmd) &&
      (prev_joy_cmd.waist_cmd == joy_cmd.waist_cmd) &&
      (prev_joy_cmd.gripper_cmd == joy_cmd.gripper_cmd) &&
      (prev_joy_cmd.pose_cmd == joy_cmd.pose_cmd) &&
      (prev_joy_cmd.speed_cmd == joy_cmd.speed_cmd) &&
      (prev_joy_cmd.speed_toggle_cmd == joy_cmd.speed_toggle_cmd) &&
      (prev_joy_cmd.gripper_pwm_cmd == joy_cmd.gripper_pwm_cmd) &&
      (prev_joy_cmd.torque_cmd == joy_cmd.torque_cmd)))
    {
      pub_joy_cmd->publish(joy_cmd);
    }
    prev_joy_cmd = joy_cmd;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InterbotixXSArmJoy>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
