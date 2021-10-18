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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;

import javax.inject.Inject;
import javax.inject.Singleton;

/**
 *  Provides the physics calculations which must be done in order
 *  to aim automatically.
 */
@Singleton
public class PhysicsUtil {
    private static final double g = 9.8 / 2;
    private final DriveTrain driveTrain;
    private final Hood hood;
    private final Turret turret;
    private final Flywheel flywheel;
    private final RSTab tab;
    private final RSTab graphTab;
    private final NavXGyro gyro;
    private final PositionTracker positionTracker;
    private double extraDistance = 0;
    private AimMode aimMode = AimMode.CALC;
    private double velocity = 9;
    private boolean autoAimEnabled = true;
    private double[] vXYZ = new double[3];

    @Inject
    public PhysicsUtil(DriveTrain driveTrain, Turret turret, Hood hood,
                       RobotShuffleboard shuffleboard, PositionTracker positionTracker,
                       NavXGyro gyro, Flywheel flywheel) {
        this.turret = turret;
        this.driveTrain = driveTrain;
        this.hood = hood;
        this.gyro = gyro;
        this.positionTracker = positionTracker;
        this.flywheel = flywheel;
        this.tab = shuffleboard.getTab("Auto Aim");
        this.graphTab = shuffleboard.getTab("Physics");
    }

    /**
     * Returns the X Value to the target depending on the vision mode selected.
     * (With a fun little "temporary" twist)
     * If aim mode is on vision it will use the lime light only, else it will use the
     * position tracker
     *
     * @return y forward distance to target x|-y
     */
    public double getX() {
        double x = aimMode != AimMode.VISION ? positionTracker.getPosition()[1] :
                getLLDistance() * Math.cos(Math.toRadians(getLLTurretAngle()));
        tab.setEntry("x", x);

        return x + extraDistance;
    }

    /**
     * Returns the Y Value to the target depending on the vision mode selected.
     * If aim mode is on vision it will use the lime light only, else it will use the
     * position tracker
     *
     * @return x horizontal distance to target x|-y
     *
     */
    public double getY() {
        double y = aimMode != AimMode.VISION ? positionTracker.getPosition()[0] :
                getLLDistance() * Math.sin(Math.toRadians(getLLTurretAngle()));
        tab.setEntry("y", y);

        return y;
    }

    /**
     * Returns the Z Vertical distance from the shooter to the target.
     * @return z vertical distance to target
     */
    public double getZ() {
        return ShooterConstants.getTopHeight();
    }

    /**
     * Returns the 2D distance to the target using the x and y values.
     *
     * @return distance to target
     */
    public double getDistanceToTarget() {
        double dist = Math.sqrt(Math.pow(getX() + extraDistance, 2) + Math.pow(getY(), 2));
        tab.setEntry("Target Dist", dist);
        return dist;
    }

    /**
     *  Returns the 2D distance to the target using the x and y values.
     *
     * @return distance to target
     */
    public double getLLTurretAngle() {
        //Returns angle to target using LL values
        double turretAngle = turret.getTurretCalculations(extraDistance, hood.getAngle())[1];
        tab.setEntry("Turret Angle", turretAngle);
        return turretAngle;
    }

    /**
     *  Returns the 2D distance to the target using the x and y values from the lime light.
     * REQUIRES DIRECT LINE OF SIGHT TO TARGET
     *
     * @return distance to target
     */
    public double getLLDistance() {
        //Returns distance to target using LL values
        double dist = turret.getTurretCalculations(extraDistance, hood.getAngle())[0];
        tab.setEntry("LL Dist", dist);
        return dist;
    }


    /**
     * Returns the angle required to launch a ball at the specified velocities given
     * through the calculateVelocities class.
     *
     * @return turret angle to launch ball at
     */
    public double getAngleToTarget() {
        //Returns angle to target using x y z position of target.
        double turretAngle = Math.toDegrees(Math.atan2(vXYZ[1], vXYZ[0]));
        tab.setEntry("Turret Angle", turretAngle);
        return turretAngle;
    }

    /**
     * Returns the angle required to launch a ball at the specified velocities given
     * through the calculateVelocities class.
     * Also includes adjustments for the hood angle which helped account for
     * inconsistencies at longer shots.
     *
     * @return hood angle to launch ball at
     */
    public double getCalculatedHoodAngle() {
        //Returns the hood angle using the relationship between horizontal and vertical velocities
        double hoodAngle = Math.toDegrees(Math.atan2(vXYZ[2], Math.sqrt(vXYZ[0] * vXYZ[0] + vXYZ[1] * vXYZ[1])));
        if (Math.abs(driveTrain.getYVelocity()) > 0.1) {
            hoodAngle += (driveTrain.getYVelocity());
        }
        double targetDist = getDistanceToTarget();

        if (targetDist < 1) {
            hoodAngle = 52; // max for close shot
        }
        tab.setEntry("Hood Angle", hoodAngle);
        return hoodAngle;
    }

    /**
     * Returns the velocity which the ball should be launched at.
     *
     * @return required ball velocity
     */
    public double getBallVel() {
        //Returns ball's velocity in m/s
        double ballVel = Math.sqrt(vXYZ[0] * vXYZ[0] + vXYZ[1] * vXYZ[1] + vXYZ[2] * vXYZ[2]);

        tab.setEntry("ballVel", ballVel);
        return ballVel;
    }

    /**
     * Does what getTurretVelocity does.
     * @see #getTurretVelocity
     */
    private double captainKalbag() {
        double targetAngle = getAngleToTarget();
        double targetTicks = turret.getTargetTicks(targetAngle, true);
        double angleDiff = turret.getPositionTicks() - targetTicks;
        //BASICALLY A PID BUT WITHOUT THE ID
        SmartDashboard.putNumber("Target ticks", targetTicks);
        SmartDashboard.putNumber("angleDiff", angleDiff);
        double p = 0.8;
        return -angleDiff * p;

    }

    /**
     * Calculates the x y and z velocities required to hit the
     * specified target. Supports two firing modes, perpendicular and normal.
     * ---------------------------------------------------------------------
     * Perpendicular shot means that the ball will be calculated to come into contact with
     * the target at a 90-degree angle. This provides a better range than our dynamic shooting mode,
     * which is locked to its physical velocity limitations.
     *----------------------------------------------------------------------
     * The dynamic shooting mode provides an accurate shot at a set velocity, meaning we
     * can achieve a much cleaner shot by picking the speed at which our turret is most accurate.
     * Using these two equations:
     * Straight Shot: sqrt(-4*g*z + 2*v^2 - 2*sqrt(-4*g^2*x^2 - 4*g^2*y^2 - 4*g*v^2*z + v^4))/(2*g)
     * Arc Shot: sqrt(2)*sqrt(-2*g*z + v^2 + sqrt(-4*g^2*x^2 - 4*g^2*y^2 - 4*g*v^2*z + v^4))/(2*g)
     * We solve for time, and translate that into a velocity which then gets subtracted from our robots
     * moving velocity.
     * This provides us with an accurate shot given any distance with our velocity, given it is within
     * the physical range of our robot.
     * It also allows us to have a much stronger PID on the flywheel, tuning it for only achieving a specific speed
     * in the quickest way possible.
     *----------------------------------------------------------------------
     * This method is also responsible for tuning the auto aim, with our 3D graph script taking data from this method
     */
    public void calculateVelocities(boolean perpendicularShot) {
        double x = getX();
        double y = getY();
        double z = getZ();
        double xVel = driveTrain.getYVelocity();
        double yVel = driveTrain.getXVelocity();

        double[] tempXYZ = { x / ShooterConstants.getTConstant() - xVel, y / ShooterConstants.getTConstant() - yVel, ShooterConstants.getZVelocityConstant() };
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
            vXYZ = !Double.isNaN(t) ? new double[]{x / t - xVel, y / t - yVel, z / t + g * t} : tempXYZ;
        } else {
            tab.setEntry("Trajectory: ", "PERP");
            graphTab.setEntry("T", ShooterConstants.getTConstant());
            vXYZ = tempXYZ;
        }

        tab.setEntry("vx", vXYZ[0]);
        tab.setEntry("vy", vXYZ[1]);
        tab.setEntry("vz", vXYZ[2]);

        graphTab.getTable("Auto Aim Stuff",
                new RSTileOptions(3, 3, 0, 0)).addTabData(tab);

        graphTab.setEntry("x", x);
        graphTab.setEntry("y", y);
        graphTab.setEntry("z", z);
        graphTab.setEntry("driveTrainVX", xVel);
        graphTab.setEntry("driveTrainVY", yVel);
        graphTab.setEntry("vx", vXYZ[0]);
        graphTab.setEntry("vy", vXYZ[1]);
        graphTab.setEntry("vz", vXYZ[2]);
        graphTab.setEntry("g", g);

        // graphTab.setEntry("turretAngle", turret.getAngle(true));
        // graphTab.setEntry("hoodAngle", hood.getAngle());
        // graphTab.setEntry("flywheelVel", flywheel.getBallVelocity());
    }

    /**
     * Returns a velocity based off of the current error in the
     * turret which should be actively updated.
     *
     * @return velocity for turret
     */
    public double getTurretVelocity() {
        return captainKalbag();
    }

    /**
     * Sets the velocity to be used by
     * calculateVelocities' calculations.
     */
    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    /**
     * Returns the target velocity for the shooter.
     * @return target velocity for the shooter
     */
    public double getTargetVelocity() {
        return velocity;
    }

    /**
     * Sets an extra amount of distance in the x direction
     * useful for the inner target.
     */
    public void setExtraDistance(double extraDistance) {
        this.extraDistance = extraDistance;
    }

    /**
     * Sets the aim mode which provides
     * different calculations for each mode.
     */
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

    public AimMode getAimMode() {
        return this.aimMode;
    }

    public enum AimMode {
        CALC, ENCODER, VISION
    }
}
