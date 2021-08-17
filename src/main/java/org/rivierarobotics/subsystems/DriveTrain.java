/*
 * This file is part of CheeseWrangler-2020, licensed under the GNU General Public License (GPLv3).
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
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.commands.drive.DriveControlCreator;
import org.rivierarobotics.inject.Sided;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.Side;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;
import javax.inject.Inject;
import javax.inject.Singleton;



/**
 * Subsystem for the drive train. Uses two component sides for drive.
 * This subsystem is used as a wrapper/holder class for the sides. Tracks
 * movement with WPILib odometry handling.
 *
 * @see DriveTrainSide
 */
@Singleton
public class DriveTrain extends SubsystemBase {
    private static final double TRACKWIDTH = 0.7366; // meters
    private final DriveTrainSide left;
    private final DriveTrainSide right;
    private final NavXGyro gyro;
    private final DifferentialDriveOdometry odometry;

    @Inject
    public DriveTrain(@Sided(Side.LEFT) DriveTrainSide left,
                      @Sided(Side.RIGHT) DriveTrainSide right,
                      NavXGyro gyro, DriveControlCreator controlCreator) {
        this.gyro = gyro;
        this.left = left;
        this.right = right;
        this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-gyro.getYaw()));
        setDefaultCommand(controlCreator.create(this));
    }

    public static double getTrackwidth() {
        return TRACKWIDTH;
    }

    public void setVelocity(double velocity) {
        setVelocity(velocity, velocity);
    }

    public void setVelocity(double l, double r) {
        left.setVelocity(l);
        right.setVelocity(r);
    }

    public void setVoltage(double l, double r) {
        left.setVoltage(l);
        right.setVoltage(r);
    }

    public void setPower(double l, double r) {
        left.setPower(l);
        right.setPower(r);
    }

    public double getAvgVelocity() {
        return (left.getVelocity() + right.getVelocity()) / 2.0;
    }

    public double getXVelocity() {
        return getAvgVelocity() * Math.sin(Math.toRadians(gyro.getYaw()));
    }

    public double getYVelocity() {
        return getAvgVelocity() * Math.cos(Math.toRadians(gyro.getYaw()));
    }

    public DriveTrainSide getLeft() {
        return left;
    }

    public DriveTrainSide getRight() {
        return right;
    }

    /**
     * Returns the pose (position and heading) as tracked by WPILib odometry
     * classes. Updated every 20ms in periodic (every loop) via shaft encoders.
     *
     * @return the current pose of the robot in meters (and radians).
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry and encoders on both drive sides. Reinitializes
     * them both with zero position and current yaw setting.
     */
    public void resetOdometry() {
        odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(-gyro.getYaw()));
        left.resetEncoder();
        right.resetEncoder();
    }

    public ArrayList<MotorTemp> getTemps() {
        ArrayList<MotorTemp> temps = new ArrayList<>();
        int index = 0;
        temps.addAll(Arrays.stream(right.getTemperature())
                .map(temp -> new MotorTemp(temp.id, temp.temp, "Right: " + temp.name))
                .collect(Collectors.toList()));
        temps.addAll(Arrays.stream(right.getTemperature())
                .map(temp -> new MotorTemp(temp.id, temp.temp, "Left: " + temp.name))
                .collect(Collectors.toList()));
        return temps;
    }

    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(-gyro.getYaw()),
            left.getPosition(),
            right.getPosition()
        );
    }
}
