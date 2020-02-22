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
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

import java.util.List;
import java.util.function.Function;

public enum Pose2dPath {
    //Add another enum entry for each path desired to enter, and a series of Pose2d objects as the points
    //Distances are in feet, x and y ordered, and angles are exit angles in degrees

    FORWARD_BACK(config -> TrajectoryGenerator.generateTrajectory(
            List.of(
                    pose2d(0, 0, 0),
                    pose2d(0.732, 1.768, 45),
                    pose2d(2.5, 2.5, 90),
                    pose2d(4.268,1.768,135),
                    pose2d(5, 0, 180),
                    pose2d(4.268, -1.768,-135),
                    pose2d(2.5, -2.5, -90),
                    pose2d(0.732,-1.768,-45),
                    pose2d(0, 0, 0)
            ),
            config
    ));

    // NOTE: X and Y swapped on PURPOSE
    // Y is the forward / back direction, X is the sideways direction

    /**
     * Given a pose in feet & degrees, generate a {@link Pose2d}.
     *
     * @param feetX x coordinate in feet
     * @param feetY y coordinate in feet
     * @param headingDegrees heading in degrees
     * @return the pose
     */
    private static Pose2d pose2d(double feetX, double feetY, double headingDegrees) {
        return new Pose2d(Units.feetToMeters(feetY), Units.feetToMeters(feetX),
            Rotation2d.fromDegrees(headingDegrees));
    }

    private static Translation2d trans2d(double feetX, double feetY) {
        return new Translation2d(Units.feetToMeters(feetY), Units.feetToMeters(feetX));
    }

    private final Function<TrajectoryConfig, Trajectory> generator;

    Pose2dPath(Function<TrajectoryConfig, Trajectory> generator) {
        this.generator = generator;
    }

    public Trajectory generateTrajectory(TrajectoryConfig config) {
        return generator.apply(config);
    }
}
