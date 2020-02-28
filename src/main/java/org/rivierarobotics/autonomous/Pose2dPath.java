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
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;

public enum Pose2dPath {
    // An underscore indicates a camelCase filename, keep all lowercase filenames and uppercase enums otherwise
    FLEX, CHEESERUN, FLEX_TAPE,
    MOVETOSHOOT, MOVETOCOLLECT, NEXTBAL, NEXTBALL, SHOOTCOLLECTED, NEXTBALLL, NEXTBALLLL, SHOOTCOLLECTEDD,
    MOVETOSHOOTT, MOVETOSHOOTTMID, MOVETOSHOOTTRIGHT;

    private final Path trajectoryJson;
    private Trajectory trajectory;

    Pose2dPath() {
        var id = name().toLowerCase(Locale.US);
        var builder = new StringBuilder(id);
        for (int i = builder.indexOf("_"); i != -1; i = builder.indexOf("_", i)) {
            builder.setCharAt(i + 1, Character.toUpperCase(builder.charAt(i + 1)));
            builder.deleteCharAt(i);
        }
        id = builder.toString();
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
