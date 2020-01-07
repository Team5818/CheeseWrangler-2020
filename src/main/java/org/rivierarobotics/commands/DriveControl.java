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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DriveTrain;

public class DriveControl extends CommandBase {
    private final DriveTrain driveTrain;
    private final Joystick leftJs, rightJs;

    public DriveControl() {
        this.driveTrain = Robot.runningRobot.driveTrain;
        this.leftJs = Robot.runningRobot.leftJs;
        this.rightJs = Robot.runningRobot.rightJs;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        if(Robot.runningRobot.isArcade) {
            double left, right;
            double x = leftJs.getX(), y = leftJs.getY();
            if (y >= 0) {
                left = y+x;
                right = y-x;
            } else {
                left = y-x;
                right = y+x;
            }
            driveTrain.setPower(left, right);
        } else {
            driveTrain.setPower(leftJs.getY(), rightJs.getY());
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
