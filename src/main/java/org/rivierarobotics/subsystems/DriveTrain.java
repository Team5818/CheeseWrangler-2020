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
import org.rivierarobotics.commands.DriveControlCreator;
import org.rivierarobotics.inject.Sided;
import org.rivierarobotics.util.NavXGyro;

import javax.inject.Inject;

public class DriveTrain implements Subsystem {
    private final DriveTrainSide left, right;
    private final NavXGyro gyro;
    private final double wheelCircumference = 0.32; // meters

    @Inject
    public DriveTrain(@Sided(Sided.Side.LEFT) DriveTrainSide left,
                      @Sided(Sided.Side.RIGHT) DriveTrainSide right,
                      NavXGyro gyro, DriveControlCreator controlCreator) {
        this.gyro = gyro;
        this.left = left;
        this.right = right;
        setDefaultCommand(controlCreator.create(this));
    }

    public void setPower(double l, double r) {
        left.setPower(l);
        right.setPower(r);
    }

    public double getAvgVelocity() {
        return (left.getVelocity() + right.getVelocity()) / 2;
    }


    //TODO this can be done without repeating anything, just pull in the yaw and determine sin/cos based on something.
    // The same thing is done to both components, so logically we can make one method for that part.
    public double getXVelocity() {
        double tickV = (getAvgVelocity() * Math.sin(Math.toRadians(gyro.getYaw())));
        return (10 * tickV * (1 / 4096.0) * wheelCircumference);
    }

    public double getYVelocity() {
        double tickV = (getAvgVelocity() * Math.cos(Math.toRadians(gyro.getYaw())));
        return (10 * tickV * (1 / 4096.0) * wheelCircumference);
    }

    public double getDistanceTicks(){
        return ( left.getPositionTicks() + right.getPositionTicks() ) / 2;
    }


    public DriveTrainSide getLeft() {
        return left;
    }

    public DriveTrainSide getRight() {
        return right;
    }
}
