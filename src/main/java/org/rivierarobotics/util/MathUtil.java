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
        angle %= 360;
        if (angle < 0) {
            return 360 + angle;
        } else {
            return angle;
        }
    }

    public static double limit(double value, double limit) {
        return limit(value, limit, limit);
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

    public static double minAbsCompare(double v1, double v2) {
        double absMin = Math.min(Math.abs(v1), Math.abs(v2));
        if (Math.abs(v1) == absMin) {
            return v1;
        } else if (Math.abs(v2) == absMin) {
            return v2;
        } else {
            return 0;
        }
    }

    public static boolean isWithinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) < tolerance;
    }

    public static double ticksPer100msToRPM(double ticksRate, double ticksPerRev) {
        return ticksPerRev * (600 / ticksPerRev);
    }

    public static double metersPerSecToTicksPer100ms(double metersPerSecond, double ticksPerMeter) {
        return metersPerSecond * (1.0 / 10) * ticksPerMeter;
    }

    public static double ticksPer100msToMetersPerSec(double ticksPer100ms, double ticksPerMeter) {
        return (ticksPer100ms * 10) / ticksPerMeter;
    }
}
