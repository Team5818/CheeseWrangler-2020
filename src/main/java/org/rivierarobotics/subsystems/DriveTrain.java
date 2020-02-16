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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.commands.DriveControlCreator;
import org.rivierarobotics.inject.Sided;
import org.rivierarobotics.util.NavXGyro;

import javax.inject.Inject;

public class DriveTrain extends SubsystemBase {
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

    public double getXVelocity() {
        double tickV = (getAvgVelocity() * Math.sin(Math.toRadians(gyro.getYaw())));
        return (10 * tickV * (1 / 4096.0) * wheelCircumference);
    }

    public double getYVelocity() {
        double tickV = (getAvgVelocity() * Math.cos(Math.toRadians(gyro.getYaw())));
        return (10 * tickV * (1 / 4096.0) * wheelCircumference);
    }

    public void setGear(Gear gear) {
        left.setGear(gear);
        right.setGear(gear);
    }

    public DriveTrainSide getLeft() {
        return left;
    }

    public DriveTrainSide getRight() {
        return right;
    }

    public NavXGyro getGyro() {
        return gyro;
    }

    public enum Gear {
        LOW, HIGH, HYBRID
    }

    public enum HybridGear {
        LOW, NORMAL, HIGH;
    }
}
