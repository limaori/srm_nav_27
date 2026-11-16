// Copyright 2026 SRM
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

#include "srm27_minco_controller/command_frame.hpp"

namespace srm27_minco_controller
{

bool resolveCommandFrame(
  const std::string & _command_frame, const std::string & _base_frame,
  const std::string & _fake_base_frame, CommandFrameMode & _mode, std::string * _reason)
{
  const auto fail = [&_reason](const std::string & _message) {
    if (_reason != nullptr) {
      *_reason = _message;
    }
    return false;
  };

  if (_command_frame.empty()) {
    return fail("command_frame must not be empty");
  }
  if (_command_frame == _base_frame) {
    _mode = CommandFrameMode::kRobotBase;
    return true;
  }
  if (_command_frame == _fake_base_frame) {
    _mode = CommandFrameMode::kOdomAlignedFake;
    return true;
  }
  // 关键：不允许"第三个名字"。命名与实际是否旋转必须一一对应，否则"到底转了几次"
  // 就成了隐含约定，而重复旋转的表现（psi≈pi/2 时前进变侧移）极难现场定位。
  return fail(
    "command_frame='" + _command_frame + "' 既不是 base_frame='" + _base_frame +
    "' 也不是 fake_base_frame='" + _fake_base_frame + "'");
}

}  // namespace srm27_minco_controller
