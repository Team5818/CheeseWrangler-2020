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

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import org.rivierarobotics.util.MathUtil;

import java.util.List;

public enum Pose2dPath {
    //Add another enum entry for each path desired to enter, and a series of Pose2d objects as the points
    //Distances are in feet, x and y ordered, and angles are exit angles in degrees

    PROVIDED_PATH(
        pose2d(-4, -1, -45),
        pose2d(-2, -2, 0),
        pose2d(0, 0, 0)
    ),

    FORWARD_BACK(
        pose2d(1, 0, 0),
        pose2d(0, 0, 0)
    ),

    SQUARE(
        pose2d(0, 4, 90),
        pose2d(4, 4, 180),
        pose2d(4, 0, 270),
        pose2d(0, 0, 0)
    ),

    TRIANGLE(
        pose2d(0, 4, 90),
        pose2d(4, 0, 45),
        pose2d(0, 0, 0)
    );

    /**
     * Given a pose in feet & degrees, generate a {@link Pose2d}.
     *
     * @param feetX x coordinate in feet
     * @param feetY y coordinate in feet
     * @param headingDegrees heading in degrees
     * @return the pose
     */
    private static Pose2d pose2d(double feetX, double feetY, double headingDegrees) {
        return new Pose2d(MathUtil.feetToMeters(feetX), MathUtil.feetToMeters(feetY),
            Rotation2d.fromDegrees(headingDegrees));
    }

    public final List<Pose2d> pointMap;

    Pose2dPath(Pose2d... pointMap) {
        this.pointMap = List.of(pointMap);
    }
}
