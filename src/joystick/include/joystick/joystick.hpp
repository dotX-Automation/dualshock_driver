/**
 * Joystick module headers.
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

#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <atomic>
#include <cfloat>
#include <chrono>
#include <cstdio>
#include <fcntl.h>
#include <linux/joystick.h>
#include <pthread.h>
#include <thread>
#include <unistd.h>

#include <dua_node/dua_node.hpp>

#include <joystick_msgs/msg/joystick.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using namespace rcl_interfaces::msg;

#define UNUSED(arg) (void)(arg)

namespace Joystick
{

enum AxisId
{
  LX,
  LY,
  L2,
  RX,
  RY,
  R2,
  DX,
  DY
};

enum AxisIdNX
{
  LXNX,
  LYNX,
  RXNX,
  L2NX,
  R2NX,
  RYNX,
  DXNX,
  DYNX
};

/**
 * Convert messages and transform data
 */
class JoystickNode : public DUANode::NodeBase
{
public:
  JoystickNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~JoystickNode();

private:
  /* Publishers */
  rclcpp::Publisher<joystick_msgs::msg::Joystick>::SharedPtr joy_pub_;

  /* Utility routines */
  int read_event(int fd, struct js_event *event);

  /* Node parameters */
  double axis_max_val;
  int64_t axis_deadzone_val;
  std::string joy_topic_name;
  std::string joy_path;

  /* Synchronization primitives for internal update operations */
  std::atomic<bool> stop_thread;

  /* Joystick thread and routine */
  std::thread joy_thread_;
  void joy_routine();
  void joy_routine_nx();

  /* Node init functions */
  void init_atomics();
  void init_parameters();
  void init_publishers();
  void init_joystick();

  /* Internal state variables */
  int js;
  struct js_event event;
  joystick_msgs::msg::Joystick joy_msg;
  std::vector<int8_t*> buttons = {&joy_msg.cross,
                                  &joy_msg.circle,
                                  &joy_msg.square,
                                  &joy_msg.triangle,
                                  &joy_msg.l1,
                                  &joy_msg.r1,
                                  &joy_msg.share,
                                  &joy_msg.options,
                                  &joy_msg.home,
                                  &joy_msg.l3,
                                  &joy_msg.r3,
                                  &joy_msg.extra1,
                                  &joy_msg.extra2,
                                  &joy_msg.extra3,
                                  &joy_msg.extra4,
                                  &joy_msg.extra5,
                                  &joy_msg.extra6,
                                  &joy_msg.extra7};
  std::vector<double*> axes = {&joy_msg.lx,
                               &joy_msg.ly,
                               &joy_msg.l2,
                               &joy_msg.rx,
                               &joy_msg.ry,
                               &joy_msg.r2,
                               &joy_msg.dx,
                               &joy_msg.dy};
};

} // namespace Joystick

#endif // JOYSTICK_HPP
