// Copyright (c) 2020 Michael Jeronimo
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <csignal>
#include <iostream>
#include <stdexcept>

#include "minipro/minipro.hpp"
#include "util/xbox360_controller.hpp"
#include "util/loop_rate.hpp"

using jeronibot::minipro::MiniPro;
using jeronibot::util::LoopRate;
using jeronibot::util::XBox360Controller;
using units::frequency::hertz;

static std::atomic<bool> should_exit{false};

void signal_handler(int signum)
{
  should_exit = true;
}

inline int sign(int a) { return a > 0 ? 1 : -1; }

//
// See https://github.com/slgrobotics/robots_bringup/tree/main/Docs/miniPRO
//

int main(int, char **)
{
  // put your miniPRO address here (use "bt-device -l"):
  const char* bt_addr = "F4:02:07:C6:C7:B4";

  try {
    signal(SIGINT, signal_handler);

    std::cout << "MiniPro: " << bt_addr << " trying to connect..." << std::endl;

    MiniPro minipro(bt_addr); // <- connection happens here
    minipro.enable_notifications();
    minipro.enter_remote_control_mode();

    std::cout << "MiniPro: connected" << std::endl;

    XBox360Controller joystick;
    LoopRate loop_rate(30_Hz);

    while (!should_exit) {
      // Flip the axis values so that forward and right are positive values
      // so that the direction of the MiniPRO matches the joysticks
      auto throttle = -joystick.get_axis_state(XBox360Controller::Axis_LeftThumbstick).y;
      auto steering = -joystick.get_axis_state(XBox360Controller::Axis_LeftThumbstick).x;

      // Set values to zero if below a specified threshold so that the MiniPRO
      // is stable when the joysticks are released (and wouldn't otherwise go
      // all the way back to 0). 4000 seems to work pretty well for my joystick
      const int zero_threshold_x = 8000;
      const int zero_threshold_y = 8000;
      if (abs(throttle) < zero_threshold_x) 
      {
        throttle = 0;
      } else {
        throttle = (abs(throttle) - zero_threshold_x) * sign(throttle);
      }

      if (abs(steering) < zero_threshold_y)
      {
        steering = 0;
      } else {
        steering = (abs(steering) - zero_threshold_y) * sign(steering);
        steering /= 10.0;  // less aggressive on turns
      }

      //std::cout << "throttle: " << throttle << "   steering: " << steering << std::endl;

      // Keep the MiniPRO fed with drive commands, throttling to achieve a
      // consistent rate. I need to empirically determine the minimum rate
      minipro.drive(throttle, steering);
      loop_rate.sleep();
    }

    // When exiting, make sure to stop the miniPRO and return to normal mode
    minipro.drive(0, 0);
    minipro.exit_remote_control_mode();
    minipro.disable_notifications();

  } catch (std::exception & ex) {
    std::cerr << "Exception: " << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
