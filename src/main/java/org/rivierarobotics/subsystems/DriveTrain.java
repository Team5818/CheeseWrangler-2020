/*
 * This file is part of Placeholder-2020, licensed under the GNU General Public License (GPLv3).
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

package org.rivierarobotics.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.rivierarobotics.commands.DriveControl;
import org.rivierarobotics.util.RobotMap;

public class DriveTrain implements Subsystem {
    private final DriveTrainSide left, right;

    public DriveTrain() {
        this.left = new DriveTrainSide(RobotMap.DriveTrain.Left.LEFT_TALON_MASTER, RobotMap.DriveTrain.Left.LEFT_SPARK_SLAVE_ONE,
                RobotMap.DriveTrain.Left.LEFT_SPARK_SLAVE_TWO, RobotMap.DriveTrain.Left.LEFT_INVERT);
        this.right = new DriveTrainSide(RobotMap.DriveTrain.Right.RIGHT_TALON_MASTER, RobotMap.DriveTrain.Right.RIGHT_SPARK_SLAVE_ONE,
                RobotMap.DriveTrain.Right.RIGHT_SPARK_SLAVE_TWO, RobotMap.DriveTrain.Right.RIGHT_INVERT);
        setDefaultCommand(new DriveControl(this));
    }

    public void setPower(double l, double r) {
        left.setManualPower(l);
        right.setManualPower(r);
    }

    public DriveTrainSide getLeft() {
        return left;
    }

    public DriveTrainSide getRight() {
        return right;
    }
}
