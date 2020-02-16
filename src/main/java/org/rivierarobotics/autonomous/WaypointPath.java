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

package org.rivierarobotics.autonomous;

import jaci.pathfinder.Waypoint;
import org.rivierarobotics.util.MathUtil;

public enum WaypointPath {
    //Add another enum entry for each path desired to enter, and a series of Waypoint objects as the points
    //Distances are in meters, x and y ordered, and angles are exit angles in radians

    PROVIDED_PATH(
            new Waypoint(MathUtil.feetToMeters(-4), MathUtil.feetToMeters(-1), Math.toRadians(-45)),
            new Waypoint(MathUtil.feetToMeters(-2), MathUtil.feetToMeters(-2), 0),
            new Waypoint(MathUtil.feetToMeters(0), MathUtil.feetToMeters(0), 0)
    ),
    SQUARE(
            new Waypoint(MathUtil.feetToMeters(0), MathUtil.feetToMeters(4), Math.toRadians(90)),
            new Waypoint(MathUtil.feetToMeters(4), MathUtil.feetToMeters(4), Math.toRadians(180)),
            new Waypoint(MathUtil.feetToMeters(4), MathUtil.feetToMeters(0), Math.toRadians(270)),
            new Waypoint(MathUtil.feetToMeters(0), MathUtil.feetToMeters(0), Math.toRadians(0))
    ),

    TRIANGLE(
            new Waypoint(MathUtil.feetToMeters(0), MathUtil.feetToMeters(4), Math.toRadians(90)),
            new Waypoint(MathUtil.feetToMeters(4), MathUtil.feetToMeters(0), Math.toRadians(45)),
            new Waypoint(MathUtil.feetToMeters(0), MathUtil.feetToMeters(0), Math.toRadians(0))
    );

    public final Waypoint[] pointMap;

    WaypointPath(Waypoint... pointMap) {
        this.pointMap = pointMap;
    }
}
