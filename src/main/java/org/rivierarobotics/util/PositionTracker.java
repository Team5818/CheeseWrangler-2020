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
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class PositionTracker {
    static double[] pos = new double[2];
    static double beforeT = 0;
    private final DriveTrain driveTrain;
    private final Hood hood;
    private final VisionUtil vision;
    private final Turret turret;
    private double t;

    @Inject
    public PositionTracker(DriveTrain dt, VisionUtil vision, Turret turret, Hood hood) {
        this.turret = turret;
        this.vision = vision;
        this.driveTrain = dt;
        this.hood = hood;
    }

    public void trackPosition() {
        t = Timer.getFPGATimestamp();
        double timeDifference = (t - beforeT);
        beforeT = Timer.getFPGATimestamp();
        pos[0] += (driveTrain.getXVelocity() * timeDifference);
        pos[1] += (driveTrain.getYVelocity() * timeDifference);
    }

    public void correctPosition() {
        if (vision.getLLValue("tv") == 0) {
            return;
        }

        double dist = ShooterUtil.getTopHeight() + ShooterUtil.getLLtoTurretY()
            / Math.tan(Math.toRadians(vision.getActualTY(hood.getAngle())));
        double turretAngle = turret.getAbsoluteAngle();
        double xFromTarget = dist * Math.sin(Math.abs(Math.toRadians(turretAngle)));
        double yFromTarget = dist * Math.cos(Math.abs(Math.toRadians(turretAngle)));
        if (turretAngle >= 0 && turretAngle < 90) {
            pos[0] = -xFromTarget;
            pos[1] = yFromTarget;
        } else if (turretAngle >= 90 && turretAngle < 180) {
            pos[0] = ShooterUtil.getLeftFieldToBallCollect() - ShooterUtil.getLeftFieldToCloseGoal() - xFromTarget;
            pos[1] = ShooterUtil.getFieldLength() - yFromTarget;
        } else if (turretAngle >= 180 && turretAngle < 270) {
            pos[0] = ShooterUtil.getLeftFieldToBallCollect() - ShooterUtil.getLeftFieldToCloseGoal() + xFromTarget;
            pos[1] = ShooterUtil.getFieldLength() - yFromTarget;
        } else {
            pos[0] = xFromTarget;
            pos[1] = yFromTarget;
        }
    }

    public void reset() {
        pos[0] = 0;
        pos[1] = 0;
    }

    public double[] getPosition() {
        return pos;
    }
}
