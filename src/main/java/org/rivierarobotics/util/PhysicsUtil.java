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
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class PhysicsUtil {
    private static final double g = 9.8 / 2;
    private final DriveTrain driveTrain;
    private final Hood hood;
    private final Turret turret;
    private final Flywheel flywheel;
    private final RobotShuffleboardTab tab;
    private final RobotShuffleboardTab graphTab;
    private final NavXGyro gyro;
    private final PositionTracker positionTracker;
    private double extraDistance = 0;
    private AimMode aimMode = AimMode.VISION;
    private double velocity = 9;
    private boolean autoAimEnabled;
    private double[] vXYZ = new double[3];

    @Inject
    public PhysicsUtil(DriveTrain dt, Turret turret, Hood hood,
                       RobotShuffleboard robotShuffleboard, PositionTracker positionTracker,
                       NavXGyro gyro, Flywheel flywheel) {
        this.turret = turret;
        this.driveTrain = dt;
        this.hood = hood;
        this.gyro = gyro;
        this.positionTracker = positionTracker;
        this.flywheel = flywheel;
        this.tab = robotShuffleboard.getTab("Auto Aim");
        this.graphTab = robotShuffleboard.getTab("Physics");
    }

    public double getX() {
        double x = aimMode != AimMode.VISION ? positionTracker.getPosition()[1] :
                getLLDistance() * Math.cos(Math.toRadians(getLLTurretAngle()));
        tab.setEntry("x", x);

        return x - x * 0.02 + extraDistance;
    }

    public double getY() {
        double y = aimMode != AimMode.VISION ? positionTracker.getPosition()[0] :
                getLLDistance() * Math.sin(Math.toRadians(getLLTurretAngle()));
        tab.setEntry("y", y);

        return y;
    }

    public double getZ() {
        return ShooterConstants.getTopHeight();
    }

    public double getDistanceToTarget() {
        double dist = Math.sqrt(Math.pow(getX() + extraDistance, 2) + Math.pow(getY(), 2));
        tab.setEntry("Target Dist", dist);
        return dist;
    }

    public double getLLTurretAngle() {
        //Returns angle to target using LL values
        double turretAngle = turret.getTurretCalculations(extraDistance, hood.getAngle())[1];
        tab.setEntry("Turret Angle", turretAngle);
        return turretAngle;
    }

    public double getLLDistance() {
        //Returns distance to target using LL values
        double dist = turret.getTurretCalculations(extraDistance, hood.getAngle())[0];
        tab.setEntry("LL Dist", dist);
        return dist;
    }

    public double getAngleToTarget() {
        //Returns angle to target using x y z position of target.
        double turretAngle = Math.toDegrees(Math.atan2(vXYZ[1], vXYZ[0]));
        tab.setEntry("Turret Angle", turretAngle);
        return turretAngle;
    }

    public double getCalculatedHoodAngle() {
        //Returns the hood angle using the relationship between horizontal and vertical velocities
        double hoodAngle = Math.toDegrees(Math.atan2(vXYZ[2], Math.sqrt(vXYZ[0] * vXYZ[0] + vXYZ[1] * vXYZ[1])));
        if (Math.abs(driveTrain.getYVelocity()) > 0.1) {
            hoodAngle += (driveTrain.getYVelocity());
        }
        tab.setEntry("Hood Angle", hoodAngle);
        return hoodAngle;
    }

    public double getHoodVel() {
        double currAng = hood.getAngle();
        double targetAng = getCalculatedHoodAngle();

        double velocityInTicksPer100ms = MathUtil.degreesToTicks((targetAng - currAng) / (0.1)) / 10;

        tab.setEntry("HoodVel", velocityInTicksPer100ms);
        return velocityInTicksPer100ms;
    }

    public double getBallVel() {
        //Returns ball's velocity in m/s
        double ballVel = Math.sqrt(vXYZ[0] * vXYZ[0] + vXYZ[1] * vXYZ[1] + vXYZ[2] * vXYZ[2]);

        tab.setEntry("ballVel", ballVel);
        return ballVel;
    }

    private double captainKalbag() {
        double targetAngle = getAngleToTarget();
        double currentAngle = (turret.getAngle(false) + gyro.getYaw()) % 360;
        double angleDiff = targetAngle - currentAngle;
        double velocityInTicksPer100ms = MathUtil.degreesToTicks((angleDiff / (0.08)) / 10);

        tab.setEntry("ActualTAngle", currentAngle);
        tab.setEntry("angleDiff", angleDiff);
        tab.setEntry("Turret Velocity: ", velocityInTicksPer100ms);

        return velocityInTicksPer100ms;
    }

    public void calculateVelocities(boolean perpendicularShot) {
        //Straight Shot: sqrt(-4*g*z + 2*v^2 - 2*sqrt(-4*g^2*x^2 - 4*g^2*y^2 - 4*g*v^2*z + v^4))/(2*g)
        //Arc Shot: sqrt(2)*sqrt(-2*g*z + v^2 + sqrt(-4*g^2*x^2 - 4*g^2*y^2 - 4*g*v^2*z + v^4))/(2*g)
        double x = getX();
        double y = getY();
        double z = getZ();
        double xVEl = driveTrain.getYVelocity();
        double yVEL = driveTrain.getXVelocity();

        double[] tempXYZ = { x / ShooterConstants.getTConstant() - xVEl, y / ShooterConstants.getTConstant() - yVEL, ShooterConstants.getZVelocityConstant() };
        if (!perpendicularShot) {
            tab.setEntry("Trajectory: ", "Curve");
            tab.setEntry("ED", extraDistance);
            double a = -4 * g * g * x * x - 4 * g * g * y * y - 4 * g * velocity * velocity * z + (velocity * velocity * velocity * velocity);
            double tStraight = Math.sqrt(-4 * g * z + 2 * (velocity * velocity) - 2 * Math.sqrt(a)) / (2 * g);
            double tArc = 1.41421 * Math.sqrt(-2 * g * z + velocity * velocity + Math.sqrt(a)) / (2 * g);
            double t = Double.isNaN(tStraight) ? tArc : tStraight;
            if (!Double.isNaN(t)) {
                graphTab.setEntry("T", t);
            } else {
                graphTab.setEntry("T", ShooterConstants.getTConstant());
            }
            if (Double.isNaN(tStraight)) {
                tab.setEntry("Trajectory: ", "ARC");
            }
            vXYZ = !Double.isNaN(t) ? new double[]{x / t - xVEl, y / t - yVEL, z / t + g * t} : tempXYZ;
        } else {
            graphTab.setEntry("T", ShooterConstants.getTConstant());
            vXYZ = tempXYZ;
        }

        tab.setEntry("vx", vXYZ[0]);
        tab.setEntry("vy", vXYZ[1]);
        tab.setEntry("vz", vXYZ[2]);

        graphTab.getTable("Auto Aim Stuff",
                new RSTOptions(3, 3, 0, 0)).addTabData(tab);

        graphTab.setEntry("x", x);
        graphTab.setEntry("y", y);
        graphTab.setEntry("z", z);
        graphTab.setEntry("driveTrainVX", xVEl);
        graphTab.setEntry("driveTrainVY", yVEL);
        graphTab.setEntry("vx", vXYZ[0]);
        graphTab.setEntry("vy", vXYZ[1]);
        graphTab.setEntry("vz", vXYZ[2]);
        graphTab.setEntry("g", g);

        graphTab.setEntry("turretAngle", turret.getAngle(true));
        graphTab.setEntry("hoodAngle", hood.getAngle());
        graphTab.setEntry("flywheelVel", flywheel.getBallVelocity());
    }

    public double getTurretVelocity() {
        return captainKalbag();
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getTargetVelocity() {
        return velocity;
    }

    public void setExtraDistance(double extraDistance) {
        this.extraDistance = extraDistance;
    }

    public void setAimMode(AimMode aimMode) {
        tab.setEntry("Aim Mode: ", aimMode.name());
        this.aimMode = aimMode;
    }

    public void toggleAutoAim() {
        autoAimEnabled = !autoAimEnabled;
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    public enum AimMode {
        CALC, ENCODER, VISION
    }
}
