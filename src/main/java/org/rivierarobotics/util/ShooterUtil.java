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

public class ShooterUtil {

    private ShooterUtil() {
    }

    public static double getTopHeight() {
        //meters
        return 1.63; //diff between robot height and top goal
    }

    public static double getDistanceFromOuterToInnerTarget() {
        //meters
        return 0.74295;
    }

    public static double getYVelocityConstant() {
        //  meters/second
        return Math.sqrt(getTopHeight() * 2 * 10);
    }

    public static double getBallMass() {
        //KG
        return 0.14;
    }

    public static double getTConstant() {
        //seconds
        return getYVelocityConstant() / 9.81;
    }

    public static double getMaxFlywheelVelocity() {
        //encoder value
        return 350;
    }

    public static double getMaxHoodAngle() {
        //degrees
        return 42;
    }

    public static double velocityToTicks(double vel) {
        return ((vel - 0.86) / .003) * (1 / 600.0) * 4.4 * 12;
    }

    public static double getFieldLength() {
        return 7.31; //meters
    }

    public static double getLeftFieldToGoal() {
        return 0.9; //meters
    }






}
