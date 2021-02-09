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

public class PathConstraints {
    private double maxAccel = SplinePath.MAX_POSSIBLE_ACCEL;
    private double maxVel = SplinePath.MAX_POSSIBLE_VEL;
    private boolean absPos = false;
    private boolean absHeading = false;
    private boolean fixedTheta = true;
    private double crKnotParam = 0.5;
    private boolean reversed = false;

    private PathConstraints() {
    }

    public PathConstraints(double maxAccel, double maxVel, boolean absPos, boolean absHeading,
                           boolean fixedTheta, double crKnotParam, boolean reversed) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.absPos = absPos;
        this.absHeading = absHeading;
        this.fixedTheta = fixedTheta;
        this.crKnotParam = crKnotParam;
        this.reversed = reversed;
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

    public PathConstraints setFixedTheta(boolean fixedTheta) {
        this.fixedTheta = fixedTheta;
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

    public boolean getFixedTheta() {
        return fixedTheta;
    }

    public double getCrKnotParam() {
        return crKnotParam;
    }

    public boolean getReversed() {
        return reversed;
    }

    public static PathConstraints create() {
        return new PathConstraints();
    }
}
