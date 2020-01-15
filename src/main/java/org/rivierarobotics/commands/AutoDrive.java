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
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.DriveTrainSide;

/**
 * An instance of InstantCommand because the setpoint only needs to be applied once
 */
public class AutoDrive extends InstantCommand
{
	private final DriveTrain driveTrain;
	private final double distance;

	/**
	 * The primary constructor for the autonomous distance travel, customizable to different distances through parameters
	 *
	 * @param inches the distance desired to travel over the course of the command
	 */
	public AutoDrive(double inches)
	{
		this.distance = inches;
		this.driveTrain = Robot.runningRobot.driveTrain;
		addRequirements(driveTrain);
	}

	/**
	 * Block only runs once. The position for one side may be different than that of the other,
	 * so the position sets must be relative to each side's current absolute position.
	 */
	@Override
	public void execute()
	{
		DriveTrainSide left = driveTrain.getLeft(), right = driveTrain.getRight();
		left.setPosition(left.getPositionInches() + distance);
		right.setPosition(right.getPositionInches() + distance);
	}
}
