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

package org.rivierarobotics.autonomous;

/**
 * PathTracer constraints for a given path. To be used with
 * {@link Pose2dPath} objects. Default values represent an optimized
 * path for either trajectory generation method with global maximums. Note
 * that global velocity maximum may exceed maximum errorless trajectory
 * following capabilities.
 *
 * @see PathTracerExecutor
 * @see Pose2dPath
 * @see CreationMode
 * @see CRKnotParam
 */
public class PathConstraints {
    private double maxAccel = SplinePath.MAX_POSSIBLE_ACCEL;
    private double maxVel = SplinePath.MAX_POSSIBLE_VEL;
    private boolean absPos = false;
    private boolean absHeading = false;
    private CreationMode creationMode = CreationMode.QUINTIC_HERMITE;
    private double crKnotParam = CRKnotParam.CENTRIPETAL.alpha();
    private boolean reversed = false;
    private boolean straight = false;

    /**
     * Use PathConstraints.create() for external object creation.
     */
    private PathConstraints() {
    }
  
    public PathConstraints(double maxAccel, double maxVel, boolean absPos, boolean absHeading,
                           CreationMode creationMode, double crKnotParam, boolean reversed, boolean straight) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.absPos = absPos;
        this.absHeading = absHeading;
        this.creationMode = creationMode;
        this.crKnotParam = crKnotParam;
        this.reversed = reversed;
        this.straight = straight;
    }

    public PathConstraints setMaxAccel(double maxAccel) {
        this.maxAccel = maxAccel;
        return this;
    }

    public PathConstraints setMaxVel(double maxVel) {
        this.maxVel = maxVel;
        return this;
    }

    public PathConstraints setAbsPos(boolean absPos) {
        this.absPos = absPos;
        return this;
    }

    public PathConstraints setAbsHeading(boolean absHeading) {
        this.absHeading = absHeading;
        return this;
    }
  
    public PathConstraints setCreationMode(CreationMode creationMode) {
        this.creationMode = creationMode;
        return this;
    }

    public PathConstraints setCrKnotParam(CRKnotParam crKnotParam) {
        this.crKnotParam = crKnotParam.alpha();
        return this;
    }

    public PathConstraints setCrKnotParam(double crKnotParam) {
        this.crKnotParam = crKnotParam;
        return this;
    }

    public PathConstraints setReversed(boolean reversed) {
        this.reversed = reversed;
        return this;
    }

    public PathConstraints setStraight(boolean straight) {
        this.straight = straight;
        return this;
    }
  
    public double getMaxAccel() {
        return maxAccel;
    }

    public double getMaxVel() {
        return maxVel;
    }

    public boolean getAbsPos() {
        return absPos;
    }

    public boolean getAbsHeading() {
        return absHeading;
    }

    public CreationMode getCreationMode() {
        return creationMode;
    }

    public double getCrKnotParam() {
        return crKnotParam;
    }

    public boolean getReversed() {
        return reversed;
    }

    public boolean getStraight() {
        return straight;
    }

    public static PathConstraints create() {
        return new PathConstraints();
    }

    /**
     * Creation mode for PathTracer trajectory generation. Each has a
     * different set of fixed/ignored values and goals. Note these are
     * representative enums for use with PathConstraints objects.
     */
    public enum CreationMode {
        /**
         * Quintic Hermite: Set tangent acceleration, velocity, and position
         * as vectors at given waypoints. Interpolates between the points to
         * create a path with constant curvature in C2.
         */
        QUINTIC_HERMITE,

        /**
         * Cubic Hermite: Same as Quintic Hermite except without
         * tangent acceleration. Constant curvature in C1. Equivalent to
         * putting 0 as acceleration tangent in Quintic Hermite.
         */
        CUBIC_HERMITE,

        /**
         * Catmull-Rom: No tangent velocity or acceleration. Use for
         * fastest path creation (waypoint positions only).
         * Interpolates between the points to create an "optimal" path with
         * smooth velocity curve. Does not guarantee reaching all waypoints.
         * Curve sharpness can be adjusted with {@link CRKnotParam}.
         * Different generation method than Hermite.
         */
        CATMULL_ROM
    }

    /**
     * Parametrization configuration for Catmull-Rom generated splines. Three
     * main types with alpha values ranging form 0 to 1. A larger value
     * equates to a sharper turn. It is suggested to use a value between 0.25
     * and 0.75 if not using one of these enums.
     */
    public enum CRKnotParam {
        /**
         * Uniform (a=0): Sharpest option, tends to make corners when
         * used with robots. May produce a loop at end point if too sharp.
         */
        UNIFORM,

        /**
         * Centripetal (a=0.5): Default option, closest to optimal
         * curved path. May deviate from exact path.
         */
        CENTRIPETAL,

        /**
         * Chordal (a=1): Large deviations from path, creates swooping
         * curves. May overshoot on robots due to turning inaccuracy.
         */
        CHORDAL;

        public double alpha() {
            return this.ordinal() / 2.0;
        }
    }
}
