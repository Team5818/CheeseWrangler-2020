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

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.rivierarobotics.commands.DriveControlCreator;
import org.rivierarobotics.inject.Sided;

import javax.inject.Inject;

public class DriveTrain implements Subsystem {
    private final DriveTrainSide left, right;
    private final PigeonIMU pigeon1;
    private final double wheelCircumference = 0.32; // meters
    double [] ypr = new double [3];

    @Inject
    public DriveTrain(@Sided(Sided.Side.LEFT) DriveTrainSide left,
                      @Sided(Sided.Side.RIGHT) DriveTrainSide right, PigeonIMU pigeon1,
                      DriveControlCreator controlCreator) {
        this.pigeon1 = pigeon1;
        this.left = left;
        this.right = right;
        setDefaultCommand(controlCreator.create(this));
    }

    public void setPower(double l, double r) {
        left.setPower(l);
        right.setPower(r);
    }

    public void resetGyro(){
        pigeon1.setYaw(0);
    }

    public double getYaw(){
        pigeon1.getYawPitchRoll(ypr);
        return(ypr[0]);
    }

    public double getAvgVelocity() {
        return (left.getVelocity() + right.getVelocity()) / 2;
    }

    public double getXVelocity() {
        double tickV = (getAvgVelocity()*Math.sin(Math.toRadians(getYaw())));
        return(10*tickV*(1/4096)*wheelCircumference);
    }

    public double getYVelocity() {
        double tickV = (getAvgVelocity() * Math.cos(Math.toRadians(getYaw())));
        return (10 * tickV * (1 / 4096) * wheelCircumference);
    }


    public DriveTrainSide getLeft() {
        return left;
    }

    public DriveTrainSide getRight() {
        return right;
    }
}
