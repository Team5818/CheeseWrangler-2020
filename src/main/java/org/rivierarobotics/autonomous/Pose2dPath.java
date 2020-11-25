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
import edu.wpi.first.wpilibj.RobotBase;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;

public enum Pose2dPath {
    // An underscore indicates a camelCase filename, keep all lowercase filenames and uppercase enums otherwise
    STRAIGHT,
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

    private Path pathFile;
    private String id;

    Pose2dPath() {
        id = name().toLowerCase(Locale.US);
        var builder = new StringBuilder(id);
        for (int i = builder.indexOf("_"); i != -1; i = builder.indexOf("_", i)) {
            builder.setCharAt(i + 1, Character.toUpperCase(builder.charAt(i + 1)));
            builder.deleteCharAt(i);
        }
        id = builder.toString();
        pathFile = getPath();
        if (!Files.exists(pathFile)) {
            throw new IllegalStateException("No path for " + id);
        }
    }

    private Path getPath() {
        var baseDir = RobotBase.isReal()
                ? Filesystem.getDeployDirectory().toPath().resolve("paths/")
                : Filesystem.getLaunchDirectory().toPath().resolve("pathWeaver/main");
        return baseDir.resolve(id);
    }

    public List<SplinePoint> getSpline() {
        try {
            List<String> rawPath = Files.readAllLines(pathFile);
            List<SplinePoint> points = new LinkedList<>();
            for (String str : rawPath.subList(1, rawPath.size())) {
                String[] values = str.split(",");
                points.add(new SplinePoint(Double.parseDouble(values[0]), Double.parseDouble(values[1]),
                        Double.parseDouble(values[2]), Double.parseDouble(values[3])));
            }
            return points;
        } catch (IOException e) {
            e.printStackTrace();
        }
        return new ArrayList<>();
    }
}
