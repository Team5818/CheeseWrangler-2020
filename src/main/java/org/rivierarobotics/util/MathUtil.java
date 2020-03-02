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
    private static final double deadband = 0.08;

    private MathUtil() {
    }

    public static double fitDeadband(double val) {
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
        return angle % 360;
    }

    public static double limit(double value, double max) {
        if (value > max) {
            return max;
        } else if (value < -max) {
            return -max;
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
}
