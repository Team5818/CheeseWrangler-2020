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

/**
 * A command for active manual driving of the bot by a driver
 * Perpetuated throughout the course of the period of the Scheduler's activity
 * Retrieves control and movement object instances from the Robot class to use
 * Requires the use of the DriveTrain
 */
public class DriveControl extends CommandBase {
    private final DriveTrain driveTrain;
    private final Joystick leftJs, rightJs;

    /**
     * Initializes local fields with runningRobot instances of DriveTrain and Joystick (x2)
     * Sets DriveTrain as requirement for this command
     */
    public DriveControl(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.leftJs = Robot.runningRobot.leftJs;
        this.rightJs = Robot.runningRobot.rightJs;
        addRequirements(driveTrain);
    }

    /**
     * Runs repeatedly throughout the course of the
     */
    @Override
    public void execute() {
        if(Robot.runningRobot.isArcade) {
            double left, right;
            double x = MathUtil.fitDeadband(leftJs.getX());
            double y = MathUtil.fitDeadband(leftJs.getY());
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

    /**
     * The command should never finish because it's always needed to control the bot,
     * hence returning false all the time would prevent it from ever finishing
     * @return if the command has finished - always false
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
