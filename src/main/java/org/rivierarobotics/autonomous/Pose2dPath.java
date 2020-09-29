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

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;

public enum Pose2dPath {
    // An underscore indicates a camelCase filename, keep all lowercase filenames and uppercase enums otherwise
    FLEX,
    FLEX_TAPE,

    TRENCH_MID_SHOOT_LOOP,
    MID_ONLY_SHOOT_LOOP,
    OUTER_SHOOT_LOOP,

    START_TOP_TO_SHOOT,
    START_MID_TO_SHOOT,
    START_BOTTOM_TO_SHOOT,
    START_TOP_TO_FAR_TRENCH,

    SHOOT_TO_LEFT_CENTER_BALL,
    SHOOT_TO_TRENCH_PICKUP,

    SHIFT_LEFT_CENTER_BALL,
    LEFT_CENTER_BALL_TO_SHOOT,
    TRENCH_END_TO_SHOOT;

    private Path trajectoryJson;
    private Trajectory trajectory;
    private String id;

    Pose2dPath() {
        id = name().toLowerCase(Locale.US);
        var builder = new StringBuilder(id);
        for (int i = builder.indexOf("_"); i != -1; i = builder.indexOf("_", i)) {
            builder.setCharAt(i + 1, Character.toUpperCase(builder.charAt(i + 1)));
            builder.deleteCharAt(i);
        }
        id = builder.toString();
        trajectoryJson = getPath();
        if (!Files.exists(trajectoryJson)) {
            throw new IllegalStateException("No path JSON for " + id);
        }
    }

    private Path getPath() {
        return Filesystem.getDeployDirectory().toPath().resolve("paths/" + id + ".wpilib.json");
    }

    public Trajectory getTrajectory() {
        if (trajectory == null) {
            trajectory = loadTrajectory(trajectoryJson);
        }
        return trajectory;
    }

    private static Trajectory loadTrajectory(Path trajectoryJson) {
        try {
            return TrajectoryUtil.fromPathweaverJson(trajectoryJson);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    public static Trajectory group(Pose2dPath... paths) {
        Trajectory traj = null;
        StringBuilder sb = new StringBuilder();
        try {
            for (Pose2dPath path : paths) {
                String fullJson = Files.readString(path.getPath(), StandardCharsets.UTF_8);
                sb.append(sb.length() == 0 ? fullJson : fullJson.substring(fullJson.indexOf("Name\n") + 4));
            }
            traj = TrajectoryUtil.deserializeTrajectory(sb.toString());
        } catch (IOException e) {
            e.printStackTrace();
        }
        return traj;
    }
}
