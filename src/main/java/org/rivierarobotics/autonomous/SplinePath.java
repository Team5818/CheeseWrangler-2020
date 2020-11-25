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

import org.rivierarobotics.subsystems.DriveTrain;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class SplinePath {
    private static final double MAX_POSSIBLE_VEL = 4.5; // m/s
    private static final double RIO_LOOP_TIME_MS = 0.02;
    private final List<SplinePoint> points;
    private final double maxVel;
    private final List<Output> precomputed;
    private final HashMap<Double, Section> sections;
    private final Output extrema;
    private double totalTime;

    public SplinePath(List<SplinePoint> points, double maxVel) {
        this.points = points;
        this.maxVel = Math.min(maxVel, MAX_POSSIBLE_VEL);
        this.precomputed = new LinkedList<>();
        this.sections = new LinkedHashMap<>();

        this.extrema = new Output(0, 0, 0, 0);
        double maxX = 0;
        double maxY = 0;
        for (SplinePoint p : points) {
            if (Math.abs(p.getX()) > maxX) {
                maxX = Math.abs(p.getX());
            }
            if (Math.abs(p.getY()) > maxY) {
                maxY = Math.abs(p.getY());
            }
        }
        extrema.setPosX(maxX);
        extrema.setPosY(maxY);

        Section sc;
        double scTime;
        for (int i = 1; i < points.size(); i++) {
            sc = new Section(points.get(i - 1), points.get(i));
            scTime = timeAtOptVel(sc, maxVel, true);
            sc.setTime(scTime);
            sections.put(totalTime, sc);
            totalTime += scTime;
        }
    }

    public SplinePath(List<SplinePoint> points) {
        this(points, MAX_POSSIBLE_VEL);
    }

    public List<SplinePoint> getPathPoints() {
        return points;
    }

    public Output calculate(double t) {
        double prev = 0;
        for (double d : sections.keySet()) {
            if (d > t) {
                break;
            } else {
                prev = d;
            }
        }
        Section curr = sections.get(prev);
        return calculate(curr, (t - prev) / curr.time);
    }

    public Output calculate(Section section, double t) {
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

        //TODO add acceleration control (h[[2], h[3])
        double[] ax = new double[2];
        double[] ay = new double[2];
        return new Output(
            h[0] * section.p0.getX() + h[1] * section.tanVX[0] + h[2] * ax[0] + h[3] * ax[1] + h[4] * section.tanVX[1] + h[5] * section.p1.getX(),
            h[0] * section.p0.getY() + h[1] * section.tanVY[0] + h[2] * ay[0] + h[3] * ay[1] + h[4] * section.tanVY[1] + h[5] * section.p1.getY(),
            hd[0] * section.p0.getX() + hd[1] * section.tanVX[0] + hd[2] * ax[0] + hd[3] * ax[1] + hd[4] * section.tanVX[1] + hd[5] * section.p1.getX(),
            hd[0] * section.p0.getY() + hd[1] * section.tanVY[0] + hd[2] * ay[0] + hd[3] * ay[1] + hd[4] * section.tanVY[1] + hd[5] * section.p1.getY()
        );
    }

    // Assumes the robot is moving with the correct trajectory
    public double[] getLRVel(Output calc) {
        return getLRVel(calc.getVelX(), calc.getVelY(), Math.atan2(calc.getPosX(), calc.getPosY()));
    }

    public double[] getLRVel(double velX, double velY, double angle) {
        double rate = Math.atan2(velX, velY);
        double vxProc = velX * Math.cos(angle) + velY * Math.sin(angle);
        return new double[] {
            vxProc - DriveTrain.getTrackwidth() / 2 * rate,
            vxProc + DriveTrain.getTrackwidth() / 2 * rate
        };
    }

    public double timeAtOptVel(Section section, double maxVel, boolean addPrecomputed) {
        double[] tempVel;
        Output tempOut;
        double maxObservedVel = 0;
        for (double t = 0; t < 1; t += RIO_LOOP_TIME_MS) {
            tempOut = calculate(section, t);
            if (addPrecomputed) {
                precomputed.add(tempOut);
            }
            tempVel = getLRVel(tempOut);
            if (Math.abs(tempVel[0]) > maxObservedVel) {
                maxObservedVel = Math.abs(tempVel[0]);
            }
            if (Math.abs(tempVel[1]) > maxObservedVel) {
                maxObservedVel = Math.abs(tempVel[1]);
            }
        }
        return maxObservedVel / maxVel;
    }

    public Output getExtrema() {
        return extrema;
    }

    public List<Output> getPrecomputedSpline() {
        return precomputed;
    }

    public Map<Double, Section> getSections() {
        return sections;
    }

    public double getMaxVel() {
        return maxVel;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public static class Section {
        private final SplinePoint p0;
        private final SplinePoint p1;
        private final double dist;
        private final double[] tanVX;
        private final double[] tanVY;
        private double time;

        public Section(SplinePoint p0, SplinePoint p1) {
            this.p0 = p0;
            this.p1 = p1;
            this.dist = Math.sqrt(Math.pow(p1.getX() - p0.getX(), 2) + Math.pow(p1.getY() - p0.getY(), 2));
            this.tanVX = new double[] {
                p0.isPrecomputedTan() ? p0.getTanVX() : Math.cos(p0.getHeading()) * dist,
                p1.isPrecomputedTan() ? p1.getTanVX() : Math.cos(p1.getHeading()) * dist
            };
            this.tanVY = new double[] {
                p0.isPrecomputedTan() ? p0.getTanVY() : Math.sin(p0.getHeading()) * dist,
                p1.isPrecomputedTan() ? p1.getTanVY() : Math.sin(p1.getHeading()) * dist
            };
        }

        public void setTime(double time) {
            this.time = time;
        }
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
            return "Output{" + "posX=" + posX + ", posY=" + posY
                    + ", velX=" + velX + ", velY=" + velY + '}';
        }
    }
}
