# -----------------------------------------------------------------------------
# The Cyberiada HSM-to-ROS2 testing environment
# -----------------------------------------------------------------------------
#
# The fake robot node: the deterministic simulation backend of the integration
# test tier. It implements the same ROS2 contract as the turtle driver, adding
# the scriptable laser scan the turtlesim simulator cannot provide.
#
# The module is a placeholder: the node contract is described in
# docs/hsm_robot_testing.md and is implemented in a separate step.
#
# Copyright (C) 2026 Alexey Fedoseev <aleksey@fedoseev.net>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see https://www.gnu.org/licenses/
#
# -----------------------------------------------------------------------------
