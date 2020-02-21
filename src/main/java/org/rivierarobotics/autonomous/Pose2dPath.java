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
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

import java.util.List;

public enum Pose2dPath {
    //Add another enum entry for each path desired to enter, and a series of Pose2d objects as the points
    //Distances are in feet, x and y ordered, and angles are exit angles in degrees

    FORWARD_BACK(
            pose2d(0, 0, 0),
            pose2d(0, 0, 0),
            trans2d(2.5, 2.5),
            trans2d(0, 5),
            trans2d(-2.5, 2.5)
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
        return new Pose2d(Units.feetToMeters(feetX), Units.feetToMeters(feetY),
            Rotation2d.fromDegrees(headingDegrees));
    }

    private static Translation2d trans2d(double feetX, double feetY) {
        return new Translation2d(Units.feetToMeters(feetX), Units.feetToMeters(feetY));
    }

    public final Pose2d start;
    public final Pose2d end;
    public final List<Translation2d> interiorWaypoints;

    Pose2dPath(Pose2d start, Pose2d end, Translation2d... interiorWaypoints) {
        this.start = start;
        this.end = end;
        this.interiorWaypoints = List.of(interiorWaypoints);
    }
}
