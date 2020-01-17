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
import org.rivierarobotics.util.MathUtil;

public class DriveControl extends CommandBase {
    private final DriveTrain driveTrain;
    private final Joystick leftJs, rightJs;

    public DriveControl(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.leftJs = Robot.runningRobot.driverLeftJs;
        this.rightJs = Robot.runningRobot.driverRightJs;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        if (Robot.runningRobot.isArcade) {
            double x = MathUtil.fitDeadband(rightJs.getX());
            double y = MathUtil.fitDeadband(leftJs.getY());
            double left, right;

            double max = Math.max(Math.abs(x), Math.abs(y));
            double diff = y - x;
            double sum = y + x;
            if (y > 0) {
               if (x > 0) {
                   left = max;
                   right = diff;
               } else {
                   left = sum;
                   right = max;
               }
            } else {
                if (x > 0) {
                    left = sum;
                    right = -max;
                } else {
                    left = -max;
                    right = diff;
                }
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