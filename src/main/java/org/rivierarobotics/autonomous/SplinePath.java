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

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class SplinePath {
    private static final double MAX_POSSIBLE_VEL = 4.5;
    private final List<SplinePoint> path;
    private final Map<Double, Output> precomputedSpline;
    private final double maxAllowedVel;
    private Output extrema;
    private double maxY;
    private double estimatedSeconds;

    public SplinePath(List<SplinePoint> path, double maxVel) {
        this.path = path;
        this.maxAllowedVel = maxVel;
        this.precomputedSpline = new LinkedHashMap<>();
        this.extrema = new Output(0, 0, 0, 0);
        if (maxVel != -1) {
            estimatedSeconds = timeAtMaxVel(maxVel);
        }
        double maxX = 0;
        double maxY = 0;
        for (SplinePoint p : path) {
            if (Math.abs(p.getX()) > maxX) {
                maxX = Math.abs(p.getX());
            }
            if (Math.abs(p.getY()) > maxY) {
                maxY = Math.abs(p.getY());
            }
        }
        extrema.setPosX(maxX);
        extrema.setPosY(maxY);
    }

    public SplinePath(List<SplinePoint> path) {
        this(path, MAX_POSSIBLE_VEL);
    }

    public List<SplinePoint> getPathPoints() {
        return path;
    }

    public Output calculate(double t) {
        return calculate(path.get((int) t), path.get((int) Math.ceil(t)), t);
    }

    public Output calculate(SplinePoint p0, SplinePoint p1, double t) {
        //TODO replace Math.pow() with t * t * t...
        t %= 1;
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
        return new Output(
            h[0] * p0.getX() + h[1] * tanVX[0] + h[2] * ax[0] + h[3] * ax[1] + h[4] * tanVX[1] + h[5] * p1.getX(),
            h[0] * p0.getY() + h[1] * tanVY[0] + h[2] * ay[0] + h[3] * ay[1] + h[4] * tanVY[1] + h[5] * p1.getY(),
            hd[0] * p0.getX() + hd[1] * tanVX[0] + hd[2] * ax[0] + hd[3] * ax[1] + hd[4] * tanVX[1] + hd[5] * p1.getX(),
            hd[0] * p0.getY() + hd[1] * tanVY[0] + hd[2] * ay[0] + hd[3] * ay[1] + hd[4] * tanVY[1] + hd[5] * p1.getY()
        );
    }

    public double[] getLRVel(double velX, double velY) {
        //TODO XY-->LR
        return new double[2];
    }

    public double timeAtMaxVel(double maxVel) {
        double[] tempVel;
        Output tempOut;
        double maxObservedVel = 0;
        for (double t = 0; t < path.size() - 1; t += 0.02) {
            tempOut = calculate(t);
            precomputedSpline.put(t, tempOut);
            tempVel = getLRVel(tempOut.getVelX(), tempOut.getVelY());
            if (tempVel[0] > maxObservedVel) {
                maxObservedVel = tempVel[0];
            }
            if (tempVel[1] > maxObservedVel) {
                maxObservedVel = tempVel[1];
            }
        }
        return maxObservedVel / maxVel;
    }

    public Output getExtrema() {
        return extrema;
    }

    public Map<Double, Output> getPrecomputedSpline() {
        return precomputedSpline;
    }

    public double getEstimatedSeconds() {
        return estimatedSeconds;
    }

    public static class Output {
        private double posX;
        private double posY;
        private double velX;
        private double velY;

        public Output(double posX, double posY, double velX, double velY) {

            this.posX = posX;
            this.posY = posY;
            this.velX = velX;
            this.velY = velY;
        }

        public void setPosX(double posX) {
            this.posX = posX;
        }

        public void setPosY(double posY) {
            this.posY = posY;
        }

        public void setVelX(double velX) {
            this.velX = velX;
        }

        public void setVelY(double velY) {
            this.velY = velY;
        }

        public double getPosX() {
            return posX;
        }

        public double getPosY() {
            return posY;
        }

        public double getVelX() {
            return velX;
        }

        public double getVelY() {
            return velY;
        }

        @Override
        public String toString() {
            return "Output{" +
                    "posX=" + posX +
                    ", posY=" + posY +
                    ", velX=" + velX +
                    ", velY=" + velY +
                    '}';
        }
    }
}
