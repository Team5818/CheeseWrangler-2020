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

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.commands.DriveControlCreator;
import org.rivierarobotics.inject.Sided;
import org.rivierarobotics.util.Dimensions;
import org.rivierarobotics.util.NavXGyro;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class DriveTrain extends SubsystemBase {
    private final DriveTrainSide left;
    private final DriveTrainSide right;
    private final NavXGyro gyro;
    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDriveOdometry odometry;

    @Inject
    public DriveTrain(@Sided(Sided.Side.LEFT) DriveTrainSide left,
                      @Sided(Sided.Side.RIGHT) DriveTrainSide right,
                      NavXGyro gyro, DriveControlCreator controlCreator) {
        this.gyro = gyro;
        this.left = left;
        this.right = right;
        this.kinematics = new DifferentialDriveKinematics(Dimensions.TRACKWIDTH);
        this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getYaw()));
        setDefaultCommand(controlCreator.create(this));
    }

    public void setVelocity(double velocity) {
        left.setVelocity(velocity);
        right.setVelocity(velocity);
    }

    public void setVelocity(double l, double r) {
        left.setVelocity(l);
        right.setVelocity(r);
    }

    public void setPower(double l, double r) {
        double rpmHighAvg = (left.getRPMHigh() + right.getRPMHigh()) / 2;
        double rpmLowAvg = (left.getRPMLow() + right.getRPMLow()) / 2;
        left.setPower(l, rpmHighAvg, rpmLowAvg);
        right.setPower(r, rpmHighAvg, rpmLowAvg);
    }

    public double getAvgVelocity() {
        return (left.getVelocity() + right.getVelocity()) / 2.0;
    }

    public double getXVelocity() {
        double tickV = (getAvgVelocity() * Math.sin(Math.toRadians(gyro.getYaw())));
        return (10 * tickV * (1 / 4096.0) * Dimensions.WHEEL_CIRCUMFERENCE);
    }

    public double getYVelocity() {
        double tickV = (getAvgVelocity() * Math.cos(Math.toRadians(gyro.getYaw())));
        return (10 * tickV * (1 / 4096.0) * Dimensions.WHEEL_CIRCUMFERENCE);
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



    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetEncoder() {
        odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(gyro.getYaw()));
        left.resetEncoder();
        right.resetEncoder();
    }

    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw()),
            Units.inchesToMeters(left.getPosition()),
            Units.inchesToMeters(right.getPosition())
        );
    }

    public enum Gear {
        LOW, HIGH, HYBRID
    }
}
