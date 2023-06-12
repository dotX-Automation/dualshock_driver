/**
 * Joystick node initialization and parameters routines.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * February 10, 2023
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <joystick/joystick.hpp>

namespace Joystick
{

/**
 * @brief Joystick node constructor.
 *
 * @param node_opts Options for the base node.
 */
JoystickNode::JoystickNode(const rclcpp::NodeOptions & node_options)
: NodeBase("joystick", node_options, true)
{
  // Initialize parameters
  init_parameters();

  // Initialize atomic members
  init_atomics();

  // Initialize topic publishers
  init_publishers();

  // Initialize joystick axes and buttons
  init_joystick();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Joystick node destructor.
 */
JoystickNode::~JoystickNode()
{
  // Stop thread and wait for it
  stop_thread.store(true, std::memory_order_release);
  joy_thread_.join();
  RCLCPP_INFO(this->get_logger(), "Thread joined correctly");
}

/**
 * @brief Routine to initialize atomic members.
 */
void JoystickNode::init_atomics()
{
  stop_thread.store(false, std::memory_order_release);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void JoystickNode::init_publishers()
{
  // Joystick
  joy_pub_ = this->create_publisher<joystick_msgs::msg::Joystick>(
    joy_topic_name,
    rclcpp::QoS(1));
}

/**
 * @brief Routine to initialize joystick buttons and axes.
 */
void JoystickNode::init_joystick()
{
  joy_thread_ = std::thread{
    &JoystickNode::joy_routine,
    this};
}

} // namespace Joystick
