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

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public enum WaypointPath {
    PROVIDED_PATH(
            new Waypoint(-4, -1, Pathfinder.d2r(-45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
            new Waypoint(-2, -2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
            new Waypoint(0, 0, 0)
    ),
    SQUARE(
            new Waypoint(0, 4, Pathfinder.d2r(90)),
            new Waypoint(4, 4, Pathfinder.d2r(180)),
            new Waypoint(4, 0, Pathfinder.d2r(270)),
            new Waypoint(0, 0, Pathfinder.d2r(0))
    );

    public final Waypoint[] pointMap;

    WaypointPath(Waypoint... pointMap) {
        this.pointMap = pointMap;
    }
}
