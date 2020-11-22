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

public class ShooterConstants {

    private static final double VEL_CONVERSION = 1077.97;

    private ShooterConstants() {
    }

    public static double getTopHeight() {
        return 1.76; //diff between robot height and top goal meters
    }

    public static double getShooterMinVelocity() {
        return 8;
    }

    public static double getShooterMaxVelocity() {
        return 19;
    }

    public static double getLLtoTurretX() {
        //meters
        return 0.203;
    }

    public static double getLLtoTurretY() {
        //meters
        //return 0.137;
        return 0;
    }

    public static double getLLtoTurretZ() {
        //meters
        //return 0.1778;
        return 0.14;
    }

    public static double getDistanceFromOuterToInnerTarget() {
        //meters
        return 0.74295;
    }

    public static double getZVelocityConstant() {
        return Math.sqrt(getTopHeight() * 2 * 9.81); //  meters/second
    }

    public static double getExtraVelocity() {
        return 0.5 * 0.2 * Math.pow(getTConstant(), 2);
    }

    public static double getBallMass() {
        return 0.14; //KG
    }

    public static double getEstimatedHoodAngle(double distance) {
        return 33 + 0.1 * distance;
    }

    public static double getTConstant() {
        return getZVelocityConstant() / 9.81; //seconds
    }

    public static double getMaxFlywheelVelocity() {
        return 19880; //encoder value
    }

    public static double velocityToTicks(double vel) {
        return (VEL_CONVERSION * vel);
    }

    public static double ticksToVelocity(double ticks) {
        return (ticks / VEL_CONVERSION);
    }

    public static double getFieldLength() {
        return 16; //meters
    }

    public static double getLeftFieldToFarGoal() {
        return 5.8; //meters
    }

    public static double getLeftFieldToCloseGoal() {
        return 2.44; // meters
    }

    public static double getLeftFieldToBallCollect() {
        return 2.778; // meters
    }
}
