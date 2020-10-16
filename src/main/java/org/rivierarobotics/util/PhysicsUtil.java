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
    private double extraDistance = 0;
    private final PositionTracker positionTracker;
    private AimMode aimMode = AimMode.VISION;
    private double velocity;
    private double[] vXYZ = {0, 0, 0};

    @Inject
    public PhysicsUtil(DriveTrain dt, VisionUtil vision, Turret turret, Hood hood, RobotShuffleboard robotShuffleboard,
                        PositionTracker positionTracker) {
        this.turret = turret;
        this.vision = vision;
        this.driveTrain = dt;
        this.hood = hood;
        this.positionTracker = positionTracker;
        this.tab = robotShuffleboard.getTab("Auto Aim");
    }

    public double getX() {
        return aimMode != AimMode.VISION ? positionTracker.getPosition()[1] :
                getLLDistance() * Math.cos(getLLTurretAngle() + extraDistance);
    }

    public double getY() {
        return aimMode != AimMode.VISION ? positionTracker.getPosition()[0] :
                getLLDistance() * Math.cos(getLLTurretAngle() + extraDistance);
    }

    public double getZ() {
        return ShooterConstants.getTopHeight();
    }

    public double getDistanceToTarget() {
        double dist = Math.sqrt(Math.pow(getX() + extraDistance, 2) + Math.pow(getY(), 2));
        tab.setEntry("Dist", dist);
        return dist;
    }

    public double getLLTurretAngle() {
        //Returns angle to target using LL values
        double turretAngle = turret.getAngle(true) + turret.getTxTurret(extraDistance, hood.getAngle());
        tab.setEntry("Turret Angle", turretAngle);
        return turretAngle;
    }

    public double getLLDistance() {
        //Returns distance to target using LL values
        double dist = ShooterConstants.getTopHeight() + ShooterConstants.getLLtoTurretY() / Math.tan(Math.toRadians(vision.getActualTY(hood.getAngle())));
        tab.setEntry("Dist", dist);
        return dist;
    }

    public double getAngleToTarget() {
        //Returns angle to target using x y z position of target.
        double turretAngle = Math.toDegrees(Math.atan2(vXYZ[1], vXYZ[2]));
        tab.setEntry("Turret Angle", turretAngle);
        return turretAngle;
    }

    public double getCalculatedHoodAngle() {
        //returns the hood angle using the relationship between horizontal and vertical velocities
        double hoodAngle = Math.toDegrees(Math.atan2(vXYZ[2], Math.sqrt(vXYZ[0] * vXYZ[0] + vXYZ[1] * vXYZ[1])));
        tab.setEntry("Hood Angle", hoodAngle);
        return hoodAngle;
    }

    public double getBallVel() {
        //Returns ball's velocity in m/s
        return Math.sqrt(vXYZ[0] * vXYZ[0] + vXYZ[1] * vXYZ[1] + vXYZ[2] * vXYZ[2]);
    }

    private double captainKalbag(double xFromGoal, double zFromGoal) {
        //Does something with derivatives idk
        double timeAdvance = 0.1;
        double xDist = xFromGoal - driveTrain.getYVelocity() * timeAdvance;
        double zDist = zFromGoal - driveTrain.getXVelocity() * timeAdvance;
        return (1 / (Math.pow((zDist / xDist), 2) + 1)) * ((-driveTrain.getXVelocity() * xDist)
                - (-driveTrain.getYVelocity() * xDist)) / Math.pow(xDist, 2);
    }

    public void calculateVelocities(boolean isArc, boolean perpendicularShot) {
        //Straight Shot: sqrt(-4*g*z + 2*v^2 - 2*sqrt(-4*g^2*x^2 - 4*g^2*y^2 - 4*g*v^2*z + v^4))/(2*g)
        //Arc Shot:sqrt(2)*sqrt(-2*g*z + v^2 + sqrt(-4*g^2*x^2 - 4*g^2*y^2 - 4*g*v^2*z + v^4))/(2*g)
        double x = getX();
        double y = getY();
        double z = getZ();
        double []testXYZ = new double[]{x / ShooterConstants.getTConstant(), y / ShooterConstants.getTConstant(), ShooterConstants.getZVelocityConstant()};
        if (!perpendicularShot) {
            tab.setEntry("Trajectory: ", "Curve or Arc");
            double g = 9.81;
            double v = velocity;
            double a = -4 * g * g * x * x - 4 * g * g * y * y - 4 * g * v * v * z + Math.pow(v, 4);
            double t = !isArc ? Math.sqrt(-4 * g * z + 2 * (v * v) - 2 * Math.sqrt(a)) / (2 * g) :
                    1.41421 * Math.sqrt(-2 * g * z + v * v + Math.sqrt(a)) / (2 * g);
            vXYZ = !Double.isNaN(t) ? new double[]{x / t, y / t, z / t + g * t} : testXYZ;
        }
        vXYZ = testXYZ;
    }

    public double getTurretVelocity() {
        return MathUtil.degreesToTicks(captainKalbag(positionTracker.getPosition()[1], positionTracker.getPosition()[0])) / 10 + 5;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
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
