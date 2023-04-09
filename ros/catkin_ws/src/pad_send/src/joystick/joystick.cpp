// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

// Modified by Alex Bethel (2022) for NMT Lunabotics:
// - Fixed unsoundness bug in move constructor.
// - Switched to exceptions rather than C-style validity querying.

#include "joystick.hpp"

#include "unistd.h"
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

Joystick::Joystick() { openPath("/dev/input/js0"); }

Joystick::Joystick(int joystickNumber) {
  std::stringstream sstm;
  sstm << "/dev/input/js" << joystickNumber;
  openPath(sstm.str());
}

Joystick::Joystick(std::string devicePath) { openPath(devicePath); }

Joystick::Joystick(std::string devicePath, bool blocking) {
  openPath(devicePath, blocking);
}

void Joystick::openPath(std::string devicePath, bool blocking) {
  // Open the device using either blocking or non-blocking
  _fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
  if (_fd < 0)
    throw string("Error opening joystick device");
}

JoystickEvent Joystick::sample() {
  JoystickEvent event;
  int bytes = read(_fd, &event, sizeof(event));

  if (bytes != sizeof(event))
    throw string("Error reading joystick event");

  return event;
}

Joystick::Joystick(Joystick &&other) {
  _fd = other._fd;
  other._fd = -1;
}

Joystick::~Joystick() {
  if (_fd >= 0)
    close(_fd);
}

std::ostream &operator<<(std::ostream &os, const JoystickEvent &e) {
  os << "type=" << static_cast<int>(e.type)
     << " number=" << static_cast<int>(e.number)
     << " value=" << static_cast<int>(e.value);
  return os;
}
