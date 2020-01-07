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

public class ButtonConfiguration {
    public static void init() {
        JoystickButton changeDriveMode = new JoystickButton(Robot.runningRobot.buttons, 1);
        changeDriveMode.whenPressed(new DriveModeSwap());

        JoystickButton autoForward = new JoystickButton(Robot.runningRobot.buttons, 2);
        autoForward.whenPressed(new AutoDrive(24));
        autoForward.whenReleased(new AutoDrive(-24));
    }
}
