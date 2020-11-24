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

import java.util.List;

public class SplinePath {
    private final List<SplinePoint> path;
    private final double maxAllowedVel;
    private double estimatedSeconds;

    public SplinePath(List<SplinePoint> path, double maxVel) {
        this.path = path;
        this.maxAllowedVel = maxVel;
        if (maxVel != -1) {
            estimatedSeconds = timeAtMaxVel(maxVel);
        }
    }

    public SplinePath(List<SplinePoint> path) {
        this(path, -1);
    }

    public double[] calculate(double t) {
        return calculate(path.get((int) t), path.get((int) Math.ceil(t)), t);
    }

    public double[] calculate(SplinePoint p0, SplinePoint p1, double t) {
        //TODO replace Math.pow() with t * t * t...
        double[] h = {
            1 - 10 * Math.pow(t, 3) + 15 * Math.pow(t, 4) - 6 * Math.pow(t, 5),
            t - 6 * Math.pow(t, 3) + 8 * Math.pow(t, 4) - 3 * Math.pow(t, 5),
            0.5 * Math.pow(t, 2) - 1.5 * Math.pow(t, 3) + 1.5 * Math.pow(t, 4) - 0.5 * Math.pow(t, 5),
            0.5 * Math.pow(t, 3) - Math.pow(t, 4) + 0.5 * Math.pow(t, 5),
            -4 * Math.pow(t, 3) + 7 * Math.pow(t, 4) - 3 * Math.pow(t, 5),
            10 * Math.pow(t, 3) - 15 * Math.pow(t, 4) + 6 * Math.pow(t, 5)
        };
        double[] hd = {
            -30 * Math.pow(t, 4) + 60 * Math.pow(t, 3) - 30 * Math.pow(t, 2),
            -15 * Math.pow(t, 4) + 32 * Math.pow(t, 3) - 18 * Math.pow(t, 2) + 1,
            -2.5 * Math.pow(t, 4) + 6 * Math.pow(t, 3) - 4.5 * Math.pow(t, 2) + t,
            2.5 * Math.pow(t, 4) - 4 * Math.pow(t, 3) + 1.5 * Math.pow(t, 2),
            -15 * Math.pow(t, 4) + 28 * Math.pow(t, 3) - 12 * Math.pow(t, 2),
            30 * Math.pow(t, 4) - 60 * Math.pow(t, 3) + 30 * Math.pow(t, 2)
        };

        double dist = Math.sqrt(Math.pow(p1.getX() - p0.getX(), 2) + Math.pow(p1.getY() - p0.getY(), 2));
        double[] tanVX = {
            p0.isPrecomputedTan() ? p0.getTanVX() : Math.cos(p0.getHeading()) * dist,
            p1.isPrecomputedTan() ? p1.getTanVX() : Math.cos(p1.getHeading()) * dist
        };
        double[] tanVY = {
            p0.isPrecomputedTan() ? p0.getTanVY() : Math.sin(p0.getHeading()) * dist,
            p1.isPrecomputedTan() ? p1.getTanVY() : Math.sin(p1.getHeading()) * dist
        };
        //TODO add acceleration control (h[[2], h[3])
        double[] ax = new double[2];
        double[] ay = new double[2];
        return new double[] {
            hd[0] * p0.getX() + hd[1] * tanVX[0] + hd[2] * ax[0] + hd[3] * ax[1] + hd[4] * tanVX[1] + hd[5] * p1.getX(), //velX
            hd[0] * p0.getY() + hd[1] * tanVY[0] + hd[2] * ay[0] + hd[3] * ay[1] + hd[4] * tanVY[1] + hd[5] * p1.getY(), //velY
            h[0] * p0.getX() + h[1] * tanVX[0] + h[2] * ax[0] + h[3] * ax[1] + h[4] * tanVX[1] + h[5] * p1.getX(), //posX
            h[0] * p0.getY() + h[1] * tanVY[0] + h[2] * ay[0] + h[3] * ay[1] + h[4] * tanVY[1] + h[5] * p1.getY(), //posY
        };
    }

    public double[] getLRVel(double[] velXY) {
        //TODO XY-->LR
        return velXY;
    }

    public double timeAtMaxVel(double maxVel) {
        double[] out;
        double maxObservedVel = 0;
        for (int t = 0; t < path.size(); t += 0.02) {
            out = getLRVel(calculate(t));
            if (out[0] > maxObservedVel) {
                maxObservedVel = out[0];
            }
            if (out[1] > maxObservedVel) {
                maxObservedVel = out[1];
            }
        }
        return maxObservedVel / maxVel;
    }

    public double getEstimatedSeconds() {
        return estimatedSeconds;
    }
}
