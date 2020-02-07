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

package org.rivierarobotics.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Turret;

import javax.inject.Inject;
import javax.inject.Singleton;

public class PositionTracker {

    private final Timer time;
    private final NavXGyro gyro;
    private final DriveTrain driveTrain;
    private final double wheelCircumference = 0.32; // meters
    private final VisionUtil vision;
    private final Turret turret;
    double[] pos = new double[2];
    double t = 0;

    @Inject
    public PositionTracker(DriveTrain dt, NavXGyro gyro, VisionUtil vision, Turret turret) {
        this.turret = turret;
        this.vision = vision;
        this.gyro = gyro;
        this.driveTrain = dt;
        time = new Timer();
    }

    public void trackPosition() {
        t = time.get() - t;
        double distanceTravelled = driveTrain.getAvgVelocity() * t;
        pos[0] = pos[0] + Math.sin(Math.toRadians(gyro.getYaw())) * distanceTravelled;
        pos[1] = pos[1] + Math.cos(Math.toRadians(gyro.getYaw())) * distanceTravelled;
        SmartDashboard.putNumber("EncoderX", pos[0]);
        SmartDashboard.putNumber("EncoderY", pos[1]);
    }

    public void correctPosition() {
        //TODO: get field length and the distance from the left side of the field to the scoring goal
        double fieldLength = 20; // add in actual measurements in meters
        double fieldWidthLeftWallToGoal = 10; // add in actual measurements in meters
        double ty = vision.getLLValue("ty");
        double h = 0.69;    //height of goal in meters
        double dist = h / Math.tan(Math.toRadians(ty)) + 0.74295;
        double tx = vision.getLLValue("tx") + Math.toRadians(turret.getAbsoluteAngle()); //returns actual tx using rotation of robot
        double txTurret = Math.atan2(dist * Math.sin(tx) + 0.1905, dist * Math.cos(tx)); //returns turret tx as it is offset from the camera.

        if (txTurret >= 0) {
            pos[0] = -dist * Math.sin(txTurret) + fieldWidthLeftWallToGoal;
        }
        else {
            pos[0] = dist * Math.sin(txTurret) + fieldWidthLeftWallToGoal;
        }
        pos[1] = -dist * Math.cos(txTurret) + fieldLength;
    }



    public double[] getPosition() {
        return pos;
    }




}
