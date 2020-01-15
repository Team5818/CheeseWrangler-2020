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

package org.rivierarobotics.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.commands.AutoDrive;
import org.rivierarobotics.commands.DriveModeSwap;
import org.rivierarobotics.commands.SetTurretPosition;

/**
 * A basic configuration of buttons based on the Joystick/GenericHID platform
 */
public class ButtonConfiguration
{
	public static void init()
	{
		// swaps the driving mode
//		JoystickButton changeDriveMode = new JoystickButton(Robot.runningRobot.coDriverRightJs, 1);
//		changeDriveMode.whenPressed(new DriveModeSwap());

		// automatically goes forwards 24 inches on press, back 24 on release
//		JoystickButton autoForward = new JoystickButton(Robot.runningRobot.coDriverRightJs, 2);
//		autoForward.whenPressed(new AutoDrive(24));
//		autoForward.whenReleased(new AutoDrive(-24));

		var firstPos = new JoystickButton(Robot.runningRobot.coDriverLeftJs, 1);
		firstPos.whenPressed(new SetTurretPosition(Robot.runningRobot.turret, 0));
		var secondPos = new JoystickButton(Robot.runningRobot.coDriverLeftJs, 2);
		secondPos.whenPressed(new SetTurretPosition(Robot.runningRobot.turret, 2048));

        /*
        // Should extend and retract piston?
        JoystickButton controlPiston = new JoystickButton(Robot.runningRobot.buttons, 3);
        controlPiston.whenPressed(new PistonControl(true));
        controlPiston.whenReleased(new PistonControl(false));
         */
	}
}