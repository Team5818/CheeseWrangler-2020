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
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;

import javax.inject.Inject;
import javax.inject.Singleton;

/**
 * Tracks the robots position on the field using
 * drive train encoders.
 */
@Singleton
public class PositionTracker {
    private final DriveTrain driveTrain;
    private final Hood hood;
    private final VisionUtil vision;
    private final Turret turret;
    private final RobotShuffleboardTab tab;
    private double[] pos = new double[2];
    private double lastTime = 0;

    @Inject
    public PositionTracker(DriveTrain driveTrain, VisionUtil vision, Turret turret,
                           Hood hood, RobotShuffleboard shuffleboard) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.turret = turret;
        this.hood = hood;
        this.tab = shuffleboard.getTab("Auto Aim");
    }

    /**
     * Must be called periodically.
     * Tracks the position of the robot using drive train velocity encoders and time passed.
     */
    public void trackPosition() {
        double timeDifference = Timer.getFPGATimestamp() - lastTime;
        lastTime = Timer.getFPGATimestamp();
        pos[0] -= driveTrain.getXVelocity() * timeDifference;
        pos[1] -= driveTrain.getYVelocity() * timeDifference;
        tab.setEntry("xFromTarget", pos[1]);
        tab.setEntry("yFromTarget", pos[0]);
    }

    /**
     * Corrects the positional readings of the robot with the LimeLight to
     * fix any errors which may have occurred due to slippage.
     */
    public void correctPosition() {
        if (vision.getLLValue("tv") == 0) {
            return;
        }
        double[] turretCalc = turret.getTurretCalculations(0, hood.getAngle());
        double dist = turretCalc[0];
        double turretAngle = turretCalc[1];
        double xFromTarget = dist * Math.sin(Math.toRadians(turretAngle));
        double yFromTarget = dist * Math.cos(Math.toRadians(turretAngle));
        pos[0] = xFromTarget;
        pos[1] = yFromTarget;
    }


    public void reset() {
        pos[0] = 0;
        pos[1] = 0;
    }

    public double[] getPosition() {
        return pos;
    }
}
