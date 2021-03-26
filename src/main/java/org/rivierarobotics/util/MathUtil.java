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

public class MathUtil {
    private static final double DEADBAND = 0.08;
    private static final double TICKS_PER_DEGREE = 4096.0 / 360;

    private MathUtil() {
    }

    public static double fitDeadband(double val) {
        return fitDeadband(val, DEADBAND);
    }

    public static double fitDeadband(double val, double deadband) {
        if (!(Math.abs(val) < deadband)) {
            if (val > 0) {
                if (val >= 1) {
                    return 1;
                } else {
                    return val - deadband;
                }
            } else if (val < 0) {
                if (val <= -1) {
                    return -1;
                } else {
                    return val + deadband;
                }
            }
        }
        return 0;
    }

    public static double wrapToCircle(double angle) {
        return wrapToCircle(angle, 360); //default 360 degrees/circle
    }

    public static double wrapToCircle(double angle, double fullCircle) {
        angle %= fullCircle;
        if (angle < 0) {
            return fullCircle + angle;
        } else {
            return angle;
        }
    }

    public static double limit(double value, double minmax) {
        return limit(value, -minmax, minmax);
    }

    public static double limit(double value, double min, double max) {
        if (value > max) {
            return max;
        } else if (value < min) {
            return min;
        } else {
            return value;
        }
    }

    public static double degreesToTicks(double degrees) {
        return degreesToTicks(degrees, TICKS_PER_DEGREE);
    }

    public static double degreesToTicks(double degrees, double ticksPerDegree) {
        return degrees * ticksPerDegree;
    }

    public static double ticksToDegrees(double ticks) {
        return ticksToDegrees(ticks, 1 / TICKS_PER_DEGREE);
    }

    public static double ticksToDegrees(double ticks, double degreesPerTick) {
        return ticks * degreesPerTick;
    }

    public static boolean isWithinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) < tolerance;
    }
}
