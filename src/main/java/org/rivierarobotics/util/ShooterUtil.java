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

public class ShooterUtil {
    //TODO:Update literally everything in this class
    private ShooterUtil() {
    }

    public static double getTopHeight() {
        return 1.78; //diff between robot height and top goal meters
    }

    public static double getMaxBallVelocity() {
        //in m/s
        return 22;
    }

    public static double getLLtoTurretX() {
        //meters
        return 0.203;
    }

    public static double getLLtoTurretY() {
        //meters
        return 0.137;
    }


    public static double getDistanceFromOuterToInnerTarget() {
        //meters
        return 0.74295;
    }

    public static double getYVelocityConstant() {
        return Math.sqrt(getTopHeight() * 2 * 10); //  meters/second
    }

    public static double getBallMass() {
        return 0.14; //KG
    }

    public static double getTConstant() {
        return getYVelocityConstant() / 9.81; //seconds
    }

    public static double getMaxFlywheelVelocity() {
        return 19880; //encoder value
    }

    public static double getMaxHoodAngle() {
        return 66; //degrees
    }

    public static double getMinHoodAngle() {
        return 33;
    }

    //TODO: Change this for the new robot once we do ball trials <- graph vel on the x and ticks per second on the y and get an equation
    public static double velocityToTicks(double vel) {
        return (1077.97 * vel - 4034);
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

}
