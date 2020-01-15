/*
 * This file is part of PracticeBot-2020-example, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.robot.Robot;

/**
 * Inverts the driving mode of the robot between arcade and tank drive
 * Stored by the instance variable isArcade in the current running robot
 * Only needs to be run once (hence InstantCommand which only runs once) to invert a boolean
 */
public class DriveModeSwap extends InstantCommand
{
	@Override
	public void execute()
	{
		Robot.runningRobot.isArcade = !Robot.runningRobot.isArcade;
	}
}
