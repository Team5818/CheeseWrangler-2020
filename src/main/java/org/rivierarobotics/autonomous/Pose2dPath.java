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

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;
import java.util.function.Function;

public enum Pose2dPath {
    FLEX, CHEESERUN, FLEXTAPE;

    private final Path trajectoryJson;
    private Trajectory trajectory;

    Pose2dPath() {
        var id = name().toLowerCase(Locale.US);
        trajectoryJson = Filesystem.getDeployDirectory().toPath().resolve("paths/" + id + ".wpilib.json");
        if (!Files.exists(trajectoryJson)) {
            throw new IllegalStateException("No path JSON for " + id);
        }
    }

    public Trajectory getTrajectory() {
        if (trajectory == null) {
            trajectory = loadTrajectory();
        }
        return trajectory;
    }

    private Trajectory loadTrajectory() {
        try {
            return TrajectoryUtil.fromPathweaverJson(trajectoryJson);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
