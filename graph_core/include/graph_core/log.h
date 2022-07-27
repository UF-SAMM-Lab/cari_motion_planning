#pragma once
/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>

/* ROS LOG WITH COLORS
   ref: https://gist.github.com/artivis/51c6c9436481b16cab2f */

namespace ros_log_color
{
  enum PRINT_COLOR
  {
    BLACK,
    RED,
    GREEN,
    YELLOW,
    BLUE,
    MAGENTA,
    CYAN,
    WHITE,
    ENDCOLOR
  };

  std::ostream& operator<<(std::ostream& os, PRINT_COLOR c);
}

#define ROS_BLACK_STREAM(x)   ROS_INFO_STREAM(ros_log_color::PRINT_COLOR::BLACK   << x << ros_log_color::PRINT_COLOR::ENDCOLOR)
#define ROS_RED_STREAM(x)     ROS_INFO_STREAM(ros_log_color::PRINT_COLOR::RED     << x << ros_log_color::PRINT_COLOR::ENDCOLOR)
#define ROS_GREEN_STREAM(x)   ROS_INFO_STREAM(ros_log_color::PRINT_COLOR::GREEN   << x << ros_log_color::PRINT_COLOR::ENDCOLOR)
#define ROS_YELLOW_STREAM(x)  ROS_INFO_STREAM(ros_log_color::PRINT_COLOR::YELLOW  << x << ros_log_color::PRINT_COLOR::ENDCOLOR)
#define ROS_BLUE_STREAM(x)    ROS_INFO_STREAM(ros_log_color::PRINT_COLOR::BLUE    << x << ros_log_color::PRINT_COLOR::ENDCOLOR)
#define ROS_MAGENTA_STREAM(x) ROS_INFO_STREAM(ros_log_color::PRINT_COLOR::MAGENTA << x << ros_log_color::PRINT_COLOR::ENDCOLOR)
#define ROS_CYAN_STREAM(x)    ROS_INFO_STREAM(ros_log_color::PRINT_COLOR::CYAN    << x << ros_log_color::PRINT_COLOR::ENDCOLOR)
#define ROS_WHITE_STREAM(x)   ROS_INFO_STREAM(ros_log_color::PRINT_COLOR::WHITE   << x << ros_log_color::PRINT_COLOR::ENDCOLOR)

