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

import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class PhysicsUtil {
    private final DriveTrain driveTrain;
    private final Hood hood;
    private final VisionUtil vision;
    private final Turret turret;
    private final RobotShuffleboardTab tab;
    private final double t = ShooterConstants.getTConstant();
    private double extraDistance = 0;
    private final PositionTracker positionTracker;
    private AimMode aimMode = AimMode.VISION;

    @Inject
    public PhysicsUtil(DriveTrain dt, VisionUtil vision, Turret turret, Hood hood, RobotShuffleboard robotShuffleboard,
                        PositionTracker positionTracker) {
        //Calc Aim
        this.turret = turret;
        this.vision = vision;
        this.driveTrain = dt;
        this.hood = hood;
        this.positionTracker = positionTracker;
        this.tab = robotShuffleboard.getTab("Auto Aim");
    }

    public double getDistanceToTarget() {
        double dist = aimMode == AimMode.VISION ? ShooterConstants.getTopHeight() / Math.tan(Math.toRadians(vision.getActualTY(hood.getAngle()))) :
                Math.sqrt(Math.pow(positionTracker.getPosition()[1] + extraDistance, 2) + Math.pow(positionTracker.getPosition()[0], 2));
        tab.setEntry("Dist", dist);
        return dist;
    }

    public double getXVelocityToTarget() {
        //goal ->   <>
        //     |
        //x -> |---- <- z
        double vx = (extraDistance + positionTracker.getPosition()[1]) / t;
        if (aimMode == AimMode.VISION) {
            vx = getDistanceToTarget() * Math.cos(getAngleToTarget() + extraDistance) / t;
        } else if (aimMode == AimMode.CALC) {
            vx -= driveTrain.getYVelocity();
        }
        tab.setEntry("X Velocity", vx);
        return vx;
    }

    public double getZVelocityToTarget() {
        //goal ->   <>
        //     |
        //x -> |---- <- z
        double vz = positionTracker.getPosition()[0] / t;
        if (aimMode == AimMode.VISION) {
            vz = getDistanceToTarget() * Math.cos(getAngleToTarget() + extraDistance) / t;
        } else if (aimMode == AimMode.CALC) {
            vz -= driveTrain.getXVelocity();
        }
        tab.setEntry("Z Velocity", vz);
        return vz;
    }

    public double getXZVelocityToTarget() {
        //goal ->    <>
        //          /
        //         /
        //xz ->   /
        double vxz = Math.sqrt(Math.pow(getXVelocityToTarget(), 2) + Math.pow(getZVelocityToTarget(), 2));
        tab.setEntry("XZ Velocity", vxz);
        return vxz;
    }

    public double getAngleToTarget() {
        //returns limelight value adjusted for offset with respect to the turrets current ABSOLUTE angle
        double angleToTarget = aimMode == AimMode.VISION ? turret.getTxTurret(getDistanceToTarget(), extraDistance) + turret.getAngle(true) :
            MathUtil.wrapToCircle(Math.toDegrees(Math.atan2(getZVelocityToTarget(), getXVelocityToTarget())));
        tab.setEntry("tx", turret.getTxTurret(getDistanceToTarget(), extraDistance));
        tab.setEntry("Turret SetAbsoluteAngle", angleToTarget);
        return angleToTarget;
    }

    public double getCalculatedHoodAngle() {
        //returns the hood angle using the relationship between horizontal and vertical velocities
        double hoodAngle = Math.toDegrees(Math.atan2(ShooterConstants.getYVelocityConstant(), getXZVelocityToTarget()));
        tab.setEntry("Hood Angle", hoodAngle);
        return hoodAngle;
    }

    public double getBallVel() {
        //Returns ball's velocity in m/s
        double ballVel = getXZVelocityToTarget() / Math.cos(Math.toRadians(getCalculatedHoodAngle()));
        if (getCalculatedHoodAngle() < MathUtil.ticksToDegrees(hood.getBackTicks())) {
            tab.setEntry("Target: ", "Close Shot");
            return ShooterConstants.getShooterMinVelocity();
        }
        tab.setEntry("Target: ", "Calculated");
        tab.setEntry("Ball Velocity", ballVel);
        return ballVel;
    }

    private double captainKalbag(double xFromGoal, double zFromGoal) {
        //Does something with derivatives idk
        double timeAdvance = 0.1;
        double xDist = xFromGoal - driveTrain.getYVelocity() * timeAdvance;
        double zDist = zFromGoal - driveTrain.getXVelocity() * timeAdvance;
        return (1 / (Math.pow((zDist / xDist), 2) + 1)) * ((-driveTrain.getXVelocity() * xDist)
                - (-driveTrain.getYVelocity() * xDist)) / Math.pow(xDist, 2);
    }

    public double getTurretVelocity() {
        return MathUtil.degreesToTicks(captainKalbag(positionTracker.getPosition()[1], positionTracker.getPosition()[0])) / 10 + 5;
    }

    public void setExtraDistance(double extraDistance) {
        this.extraDistance = extraDistance;
    }

    public void setAimMode(AimMode aimMode) {
        tab.setEntry("Aim Mode: ", aimMode.name());
        this.aimMode = aimMode;
    }

    public enum AimMode {
        CALC, ENCODER, VISION
    }
}
