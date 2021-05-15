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

/**
 * Utility methods relating to robot mathematics.
 */
public class MathUtil {
    private static final double DEADBAND = 0.08;
    private static final double TICKS_PER_DEGREE = 4096.0 / 360;

    private MathUtil() {
    }

    /**
     * Fits value between -1 and 1 but eliminates noise between the deadband.
     * Uses a system default value for the deadband.
     *
     * <p>Wrapper for {@link #fitDeadband(double, double)}.</p>
     *
     * @param val the value to fit inside the deadband.
     * @return the value fitted to the range.
     *
     * @see #fitDeadband(double, double)
     */
    public static double fitDeadband(double val) {
        return fitDeadband(val, DEADBAND);
    }

    /**
     * Fits value between -1 and 1 but eliminates noise between the deadband.
     *
     * <p>Generally used for eliminating noise in joystick readings by setting the
     * output to zero when within a certain deadband amount of zero. Non-joystick
     * values are also limited between -1 and 1 so as to use in a motor set.</p>
     *
     * @param val the value to fit inside the valid range and outside the deadband.
     * @param deadband the amount of tolerance around zero in which
     *                 values are set to zero.
     * @return the value fitted to the range.
     */
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

    /**
     * Wraps a circular angle value to within one circle. Uses a standard
     * 360 degrees per circle, angle is passed in degrees.
     *
     * <p>Wrapper for {@link #wrapToCircle(double, double)}</p>
     *
     * @param angle the number of degrees to wrap.
     * @return the [0, 360] degrees angle after being wrapped.
     *
     * @see #wrapToCircle(double, double)
     */
    public static double wrapToCircle(double angle) {
        return wrapToCircle(angle, 360);
    }

    /**
     * Wraps a circular angle value to within one circle.
     *
     * <p>Customizable number of angle units per circle. Angle is
     * equivalent and wrapped to the positive [0, fullCircle] range.</p>
     *
     * @param angle the raw angle units to wrap.
     * @param fullCircle the number of angle units to be one circle.
     * @return the wrapped angle in corresponding angle units.
     */
    public static double wrapToCircle(double angle, double fullCircle) {
        angle %= fullCircle;
        if (angle < 0) {
            return fullCircle + angle;
        } else {
            return angle;
        }
    }

    /**
     * Fits a value between an upper and lower limit.
     * Creates the lower limit by negating the passed value.
     *
     * <p>Wrapper for {@link #limit(double, double, double)}.</p>
     *
     * @see #limit(double, double, double)
     */
    public static double limit(double value, double minmax) {
        return limit(value, -minmax, minmax);
    }

    /**
     * Fits a value between an upper and lower limit.
     *
     * @param value the value to fit between the limits.
     * @param min the minimum value that the output may have, inclusive.
     * @param max the maximum value that the output may have, inclusive.
     * @return the value fitted between the minimum and maximum limits.
     */
    public static double limit(double value, double min, double max) {
        if (value > max) {
            return max;
        } else if (value < min) {
            return min;
        } else {
            return value;
        }
    }

    /**
     * Converts degrees angles to ticks values.
     * Uses a standard 4096 ticks per circle.
     *
     * <p>Wrapper for {@link #degreesToTicks(double, double)}.</p>
     *
     * @see #degreesToTicks(double, double)
     */
    public static double degreesToTicks(double degrees) {
        return degreesToTicks(degrees, TICKS_PER_DEGREE);
    }

    /**
     * Converts degrees angles to ticks values.
     * Customizable number of ticks per degree.
     *
     * @param degrees angle to convert into ticks.
     * @param ticksPerDegree number of ticks per degree.
     * @return the degrees angle as a tick value.
     */
    public static double degreesToTicks(double degrees, double ticksPerDegree) {
        return degrees * ticksPerDegree;
    }

    /**
     * Converts ticks values to degrees angles.
     * Uses a standard 4096 ticks per circle.
     *
     * <p>Wrapper for {@link #ticksToDegrees(double, double)}.</p>
     *
     * @see #ticksToDegrees(double, double)
     */
    public static double ticksToDegrees(double ticks) {
        return ticksToDegrees(ticks, 1 / TICKS_PER_DEGREE);
    }

    /**
     * Converts ticks values to degrees angles.
     * Customizable number of degrees per tick.
     *
     * @param ticks tick value to convert into degrees.
     * @param degreesPerTick number of degrees per tick.
     * @return the ticks value as a degrees angle.
     */
    public static double ticksToDegrees(double ticks, double degreesPerTick) {
        return ticks * degreesPerTick;
    }

    /**
     * Checks if a value is within a certain tolerance of a target. Directions irrelevant.
     *
     * @param value the current value for which to check.
     * @param target the target to check the value against.
     * @param tolerance the tolerance (positive and negative directions)
     *                  around the target that is acceptable error
     *                  for the value to be "within tolerance".
     * @return if the value is within tolerance of the target.
     */
    public static boolean isWithinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) < tolerance;
    }

    /**
     * Calculates bounded motor outputs for an arcade drive system.
     *
     * <p>Turns using the x input and fwd/back using the y.
     * Generally used to emulate the feel of driving in an "arcade"
     * instead of standard tank drive (west coast motor configuration).
     * May pass any input, most often used with two joysticks:
     * left driver Y axis and right driver X axis.</p>
     *
     * @param x joystick input [-1, 1] used for turning.
     * @param y joystick input [-1, 1] used for forward/backward movement.
     * @return power set [-1, 1] for left (index 0)
     *         and right (index 1) drive sides
     */
    public static double[] arcadeDrive(double x, double y) {
        double left;
        double right;

        double max = Math.max(Math.abs(x), Math.abs(y));
        double diff = y - x;
        double sum = y + x;
        if (y > 0) {
            if (x > 0) {
                left = max;
                right = diff;
            } else {
                left = sum;
                right = max;
            }
        } else {
            if (x > 0) {
                left = sum;
                right = -max;
            } else {
                left = -max;
                right = diff;
            }
        }
        return new double[] {
            left, right
        };
    }
}
