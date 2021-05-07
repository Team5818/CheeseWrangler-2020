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
import org.rivierarobotics.util.Vec2D;

/**
 * Generic class for storing a spline vector. Uses <code>Vec2D</code> logic
 * backing. May use precomputed tangents (Quintic Hermite spline generation
 * only) or a simple heading. Stored in metric units (meters/second or radians).
 *
 * @see Vec2D
 */
public class SplinePoint extends Vec2D {
    private final boolean precomputedTan;
    private double headingRadians;
    private double vxTan;
    private double vyTan;

    /**
     * Constructs a storage object. To be used with Quintic Hermite spline
     * generation only due to precomputed tangent velocities.
     *
     * @param x the x coordinate of this waypoint in meters.
     * @param y the y coordinate of this waypoint in meters.
     * @param vxTan the tangent velocity at this waypoint in the x direction.
     * @param vyTan the tangent velocity at this waypoint in the y direction.
     */
    public SplinePoint(double x, double y, double vxTan, double vyTan) {
        super(x, y);
        this.vxTan = vxTan;
        this.vyTan = vyTan;
        this.precomputedTan = true;
    }

    /**
     * Constructs a storage object. May be used with any other trajectory
     * generation method. No precomputed tangent velocities.
     *
     * @param x the x coordinate of this waypoint in meters.
     * @param y the y coordinate of this waypoint in meters.
     * @param headingDegrees the heading of this point in degrees.
     */
    public SplinePoint(double x, double y, double headingDegrees) {
        super(x, y);
        this.headingRadians = Math.toRadians(headingDegrees);
        this.precomputedTan = false;
    }

    public SplinePoint(Pose2d pose) {
        this(pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getRotation().getDegrees());
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
