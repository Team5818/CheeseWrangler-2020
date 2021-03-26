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

import java.util.LinkedList;
import java.util.List;

public enum ChallengePath {
    // Galactic search (2.4.6)
    GS_A_RED("C1", "C3", "D5", "A6", "A11"),
    GS_A_BLUE("C1", "E6", "B7", "C9", "D11"),
    GS_B_RED("C1", "B3", "D5", "B7", "A11"),
    GS_B_BLUE("C1", "D6", "B8", "D10", "E11"),

    // AutoNav (2.4.7)
    AN_BARREL_RACING(),
    AN_SLALOM(),
    AN_BOUNCE();

    private static final double METERS_PER_GRID = 0.762;
    private final SplinePath path;

    /**
     * Converts control strings into PathTracer paths for FIRST at Home challenges.
     * Figure 2-2 (2021 FAH manual) for layout. Use upper case letters for rows.
     * @param constraints the constraints for the path, always non-fixed theta
     * @param controlSeq a sequence of 2-char control strings formatted as [A:E][1:11].
     */
    ChallengePath(PathConstraints constraints, String... controlSeq) {
        constraints = constraints.setFixedTheta(false);
        List<SplinePoint> points = new LinkedList<>();
        for (String control : controlSeq) {
            if (control.length() < 2) {
                invalidControlError(control);
            }
            control = control.toUpperCase();
            int row = ((int) control.charAt(0)) - 64;
            int col = Integer.parseInt(control.substring(1));
            if (row < 1 || row > 5 || col < 1 || col > 11) {
                invalidControlError(control);
            }
            // Heading degrees doesn't matter b/c non fixed theta, reverse y b/c PathWeaver
            points.add(new SplinePoint(METERS_PER_GRID * col, -METERS_PER_GRID * row, 0));
        }
        this.path = new SplinePath(points, constraints);
    }

    ChallengePath(String... controlSeq) {
        this(PathConstraints.create(), controlSeq);
    }

    /**
     * Wrapper constructor for PathWeaver generated splines.
     * Use if control sequences are not precise enough.
     * @param path preexisting Pose2dPath generated by PathWeaver GUI.
     */
    ChallengePath(Pose2dPath path) {
        this.path = new SplinePath(path.getSpline());
    }

    public SplinePath getPath() {
        return path;
    }

    private void invalidControlError(String control) {
        throw new IllegalArgumentException("Control sequences must be of type [A:E][1:11]. Given: " + control);
    }

    // Knot parameterization types. Use alpha() with PathConstraints.
    private enum CRKnotParam {
        UNIFORM, CENTRIPETAL, CHORDAL;

        public double alpha() {
            return this.ordinal() / 2.0;
        }
    }
}