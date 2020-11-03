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
    private static final double g = 9.81 / 2;
    private final DriveTrain driveTrain;
    private final Hood hood;
    private final VisionUtil vision;
    private final Turret turret;
    private final RobotShuffleboardTab tab;
    private final RobotShuffleboardTab testTab;
    private final PositionTracker positionTracker;
    private double extraDistance = 0;
    private AimMode aimMode = AimMode.VISION;
    private static double velocity = 19;
    private static boolean autoAimEnabled;
    private double[] vXYZ = new double[3];

    @Inject
    public PhysicsUtil(DriveTrain dt, VisionUtil vision, Turret turret, Hood hood,
                       RobotShuffleboard robotShuffleboard, PositionTracker positionTracker) {
        this.turret = turret;
        this.vision = vision;
        this.driveTrain = dt;
        this.hood = hood;
        this.positionTracker = positionTracker;
        this.tab = robotShuffleboard.getTab("Auto Aim");
        this.testTab = robotShuffleboard.getTab("Test3DGrapher");
    }

    public double getX() {
        double x = aimMode != AimMode.VISION ? positionTracker.getPosition()[1] :
                getLLDistance() * Math.cos(Math.toRadians(getLLTurretAngle()));
        tab.setEntry("x", x);
        return x;
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
        tab.setEntry("Hood Angle", hoodAngle);
        return hoodAngle;
    }

    public double getBallVel() {
        //Returns ball's velocity in m/s
        double ballVel = Math.sqrt(vXYZ[0] * vXYZ[0] + vXYZ[1] * vXYZ[1] + vXYZ[2] * vXYZ[2]);
        tab.setEntry("ballVel", ballVel);
        return ballVel;
    }

    public double getBallVel(double hoodAngle) {
        //Returns ball's velocity in m/s
        double t = 0.7;
        return (getZ() / t + g * t) / Math.sin(Math.toRadians(hoodAngle));
    }

    private double captainKalbag() {
        //Equation: (vx*y - vy*x)/((vx^2 + vy^2)*t^2 + (-2*vx*x - 2*vy*y)*t + x^2 + y^2)
        //Returns change in ticks per 100ms
        double t = 0;
        double x = getX();
        double y = getY();
        double vx = driveTrain.getXVelocity();
        double vy = driveTrain.getYVelocity();
        double velocityInRads = (vx * y - vy * x) / ((vx * vx + vy * vy) * t * t + (-2 * vx * x - 2 * vy * y) * t + x * x + y * y);
        double velocityInDegrees = velocityInRads * (180 / Math.PI);
        double velocityInTicksPer100ms = MathUtil.degreesToTicks(velocityInDegrees) / 10;
        tab.setEntry("Turret Velocity: ", velocityInTicksPer100ms);
        return velocityInTicksPer100ms;
    }

    public void calculateVelocities(boolean perpendicularShot) {
        //Straight Shot: sqrt(-4*g*z + 2*v^2 - 2*sqrt(-4*g^2*x^2 - 4*g^2*y^2 - 4*g*v^2*z + v^4))/(2*g)
        //Arc Shot:sqrt(2)*sqrt(-2*g*z + v^2 + sqrt(-4*g^2*x^2 - 4*g^2*y^2 - 4*g*v^2*z + v^4))/(2*g)
        double x = getX();
        double y = getY();
        double z = getZ();
        double[] tempXYZ = { x / ShooterConstants.getTConstant(), y / ShooterConstants.getTConstant(), ShooterConstants.getZVelocityConstant() };
        if (!perpendicularShot) {
            tab.setEntry("Trajectory: ", "Curve");
            double a = -4 * g * g * x * x - 4 * g * g * y * y - 4 * g * velocity * velocity * z + (velocity * velocity * velocity * velocity);
            double tStraight = Math.sqrt(-4 * g * z + 2 * (velocity * velocity) - 2 * Math.sqrt(a)) / (2 * g);
            double tArc = 1.41421 * Math.sqrt(-2 * g * z + velocity * velocity + Math.sqrt(a)) / (2 * g);
            double t = Double.isNaN(tStraight) ? tArc : tStraight;
            testTab.setEntry("T", t);
            if (Double.isNaN(tStraight)) {
                tab.setEntry("Trajectory: ", "ARC");
            }
            vXYZ = !Double.isNaN(t) ? new double[]{x / t, y / t, z / t + g * t} : tempXYZ;
        } else {
            testTab.setEntry("T", ShooterConstants.getTConstant());
            vXYZ = tempXYZ;
        }
        tab.setEntry("vx", vXYZ[0]);
        tab.setEntry("vy", vXYZ[1]);
        tab.setEntry("vz", vXYZ[2]);

        testTab.setEntry("vx", vXYZ[0]);
        testTab.setEntry("vy", vXYZ[1]);
        testTab.setEntry("vz", vXYZ[2]);
        testTab.setEntry("g", g);
    }

    public double getTurretVelocity() {
        return captainKalbag();
    }

    public void setVelocity(double velocity) {
        PhysicsUtil.velocity = velocity;
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
