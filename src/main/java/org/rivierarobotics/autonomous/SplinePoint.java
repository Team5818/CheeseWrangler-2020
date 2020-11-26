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

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class SplinePoint {
    private final double x;
    private final double y;
    private final boolean precomputedTan;
    private double headingRadians;
    private double vxTan;
    private double vyTan;

    public SplinePoint(double x, double y, double vxTan, double vyTan) {
        this.x = x;
        this.y = y;
        this.vxTan = vxTan;
        this.vyTan = vyTan;
        this.precomputedTan = true;
    }

    public SplinePoint(double x, double y, double headingDegrees) {
        this.x = x;
        this.y = y;
        this.headingRadians = Math.toRadians(headingDegrees);
        this.precomputedTan = false;
    }

    public SplinePoint(Pose2d pose) {
        this(pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getRotation().getDegrees());
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTanVX() {
        return vxTan;
    }

    public double getTanVY() {
        return vyTan;
    }

    public double getHeading() {
        return headingRadians;
    }

    public boolean isPrecomputedTan() {
        return precomputedTan;
    }
}
