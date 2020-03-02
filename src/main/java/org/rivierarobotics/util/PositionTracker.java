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

package org.rivierarobotics.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Turret;

import javax.inject.Inject;

public class PositionTracker {
    private final Timer time;
    private final NavXGyro gyro;
    private final DriveTrain driveTrain;
    private final double wheelCircumference = 0.32; // meters
    private final VisionUtil vision;
    private final Turret turret;
    static double[] pos = new double[2];
    static double beforeT = 0;
    double t;

    @Inject
    public PositionTracker(DriveTrain dt, NavXGyro gyro, VisionUtil vision, Turret turret) {
        this.turret = turret;
        this.vision = vision;
        this.gyro = gyro;
        this.driveTrain = dt;
        time = new Timer();
    }

    public void trackPosition() {
        SmartDashboard.putNumber("before", beforeT);
        t = Timer.getFPGATimestamp();
        double timeDifference = (t - beforeT);
        SmartDashboard.putNumber("change in time", t - beforeT);
        beforeT = Timer.getFPGATimestamp();
        pos[0] = pos[0] + driveTrain.getXVelocity() * timeDifference;
        pos[1] = pos[1] + driveTrain.getYVelocity() * timeDifference;
        SmartDashboard.putNumber("EncoderX", pos[0]);
        SmartDashboard.putNumber("EncoderY", pos[1]);
    }

    public void correctPosition() {
        //TODO: get field length and the distance from the left side of the field to the scoring goal
        double fieldLength = ShooterUtil.getFieldLength();
        double fieldWidthLeftWallToGoal = ShooterUtil.getLeftFieldToGoal();
        double dist = ShooterUtil.getTopHeight() / Math.tan(Math.toRadians(vision.getLLValue("ty")));
        double txTurret = turret.getTxTurret(dist, 0); //returns turret tx as it is offset from the camera.
        if (txTurret >= 0) {
            pos[0] = fieldWidthLeftWallToGoal - dist * Math.sin(txTurret);
        } else {
            pos[0] = dist * Math.sin(Math.abs(txTurret)) + fieldWidthLeftWallToGoal;
        }
        pos[1] = (fieldLength) - dist * Math.cos(txTurret);
    }

    public double[] getPosition() {
        return pos;
    }
}
