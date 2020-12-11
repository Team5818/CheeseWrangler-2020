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
import org.rivierarobotics.util.Pair;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class SplinePath {
    private static final double MAX_POSSIBLE_VEL = 4.5; // m/s
    private static final double MAX_POSSIBLE_ACCEL = 2.0; // m/s^2
    private static final double RIO_LOOP_TIME_MS = 0.02;
    private final List<SplinePoint> points;
    private final double maxVel;
    private final double maxAccel;
    private final List<SPOutput> precomputed;
    private final HashMap<Double, Section> sections;
    private Pair<Double> extrema;
    private double totalTime;

    public SplinePath(List<SplinePoint> points, double maxVel, double maxAccel) {
        this.points = points;
        this.maxVel = Math.min(maxVel, MAX_POSSIBLE_VEL);
        this.maxAccel = Math.min(maxAccel, MAX_POSSIBLE_ACCEL);
        this.precomputed = new LinkedList<>();
        this.sections = new LinkedHashMap<>();

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
        this.extrema = new Pair<>(maxX, maxY);
        recalculatePath();
    }

    public SplinePath(List<SplinePoint> points) {
        this(points, MAX_POSSIBLE_VEL, MAX_POSSIBLE_ACCEL);
    }

    public void recalculatePath() {
        totalTime = 0;
        sections.clear();
        precomputed.clear();
        Section sc;
        double scTime;
        for (int i = 1; i < points.size(); i++) {
            sc = new Section(points.get(i - 1), points.get(i));
            scTime = timeAtLimited(sc, true);
            sc.setTime(scTime);
            sc.setAccel(calculate(sc, 0, false), calculate(sc, 1 - RIO_LOOP_TIME_MS, false));
            sections.put(totalTime, sc);
            totalTime += scTime;
        }
    }

    public SPOutput calculate(double t) {
        double prev = 0;
        for (double d : sections.keySet()) {
            if (d > t) {
                break;
            } else {
                prev = d;
            }
        }
        Section curr = sections.get(prev);
        return calculate(curr, (t - prev) / curr.time, true);
    }

    public SPOutput calculate(Section section, double t, boolean timeCorrected) {
        double[] td = {
            1, t, t * t, t * t * t, t * t * t * t, t * t * t * t * t
        };
        double[] h = {
            1 - 10 * td[3] + 15 * td[4] - 6 * td[5],
            t - 6 * td[3] + 8 * td[4] - 3 * td[5],
            0.5 * td[2] - 1.5 * td[3] + 1.5 * td[4] - 0.5 * td[5],
            0.5 * td[3] - td[4] + 0.5 * td[5],
            -4 * td[3] + 7 * td[4] - 3 * td[5],
            10 * td[3] - 15 * td[4] + 6 * td[5]
        };
        double[] hd = {
            -30 * td[4] + 60 * td[3] - 30 * td[2],
            -15 * td[4] + 32 * td[3] - 18 * td[2] + 1,
            -2.5 * td[4] + 6 * td[3] - 4.5 * td[2] + t,
            2.5 * td[4] - 4 * td[3] + 1.5 * td[2],
            -15 * td[4] + 28 * td[3] - 12 * td[2],
            30 * td[4] - 60 * td[3] + 30 * td[2]
        };
        double[] hdd = {
            -120 * td[3] + 180 * td[2] - 60 * t,
            -60 * td[3] + 96 * td[2] - 36 * t,
            -10 * td[3] + 18 * td[2] - 9 * t + 1,
            10 * td[3] - 12 * td[2] + 3 * t,
            -60 * td[3] + 84 * td[2] - 24 * t,
            120 * td[3] - 180 * td[2] + 60 * t
        };

        double time = timeCorrected ? section.time : 1;
        return new SPOutput(
            h[0] * section.p0.getX() + h[1] * section.tanVX[0] + h[2] * section.accelX[0] + h[3] * section.accelX[1] + h[4] * section.tanVX[1] + h[5] * section.p1.getX(),
            h[0] * section.p0.getY() + h[1] * section.tanVY[0] + h[2] * section.accelY[0] + h[3] * section.accelY[1] + h[4] * section.tanVY[1] + h[5] * section.p1.getY(),
            (hd[0] * section.p0.getX() + hd[1] * section.tanVX[0] + hd[2] * section.accelX[0] + hd[3] * section.accelX[1] + hd[4] * section.tanVX[1] + hd[5] * section.p1.getX()) / time,
            (hd[0] * section.p0.getY() + hd[1] * section.tanVY[0] + hd[2] * section.accelY[0] + hd[3] * section.accelY[1] + hd[4] * section.tanVY[1] + hd[5] * section.p1.getY()) / time,
            (hdd[0] * section.p0.getX() + hdd[1] * section.tanVX[0] + hdd[2] * section.accelX[0] + hdd[3] * section.accelX[1] + hdd[4] * section.tanVX[1] + hdd[5] * section.p1.getX()) / time,
            (hdd[0] * section.p0.getY() + hdd[1] * section.tanVY[0] + hdd[2] * section.accelY[0] + hdd[3] * section.accelY[1] + hdd[4] * section.tanVY[1] + hdd[5] * section.p1.getY()) / time
        );
    }

    // Assumes the robot is moving with the correct trajectory
    public Pair<Double> getLRVel(SPOutput calc) {
        return getLRVel(calc.getVelX(), calc.getVelY(), Math.atan2(calc.getPosX(), calc.getPosY()));
    }

    public Pair<Double> getLRVel(double velX, double velY, double angle) {
        double rate = Math.atan2(velX, velY);
        double vxProc = velX * Math.cos(angle) + velY * Math.sin(angle);
        return new Pair<>(
            vxProc - DriveTrain.getTrackwidth() / 2 * rate,
            vxProc + DriveTrain.getTrackwidth() / 2 * rate
        );
    }

    public double timeAtLimited(Section section, boolean addPrecomputed) {
        Pair<Double> tempVel;
        Pair<Double> tempAccel;
        SPOutput tempOut;
        double maxObservedVel = 0;
        double maxObservedAccel = 0;
        for (double t = 0; t < 1; t += RIO_LOOP_TIME_MS) {
            tempOut = calculate(section, t, false);
            if (addPrecomputed) {
                precomputed.add(tempOut);
            }

            tempVel = getLRVel(tempOut);
            if (Math.abs(tempVel.getA()) > maxObservedVel) {
                maxObservedVel = Math.abs(tempVel.getA());
            }
            if (Math.abs(tempVel.getB()) > maxObservedVel) {
                maxObservedVel = Math.abs(tempVel.getB());
            }

            // Just use accel/vel instead of vel/pos (?)
            tempAccel = getLRVel(tempOut.getAccelX(), tempOut.getAccelY(),
                    Math.atan2(tempOut.getVelX(), tempOut.getVelY()));
            if (Math.abs(tempAccel.getA()) > maxObservedAccel) {
                maxObservedAccel = Math.abs(tempAccel.getA());
            }
            if (Math.abs(tempAccel.getB()) > maxObservedAccel) {
                maxObservedAccel = Math.abs(tempAccel.getB());
            }
        }
        return ((maxObservedVel / maxVel) + (maxObservedAccel / maxAccel)) / 2;
    }

    public Pair<Double> getExtrema() {
        return extrema;
    }

    public List<SPOutput> getPrecomputedSpline() {
        return precomputed;
    }

    public List<SplinePoint> getPathPoints() {
        return points;
    }

    public void addPathPoint(int idx, SplinePoint point) {
        points.add(idx, point);
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
        private double[] accelX;
        private double[] accelY;
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
            this.accelX = new double[2];
            this.accelY = new double[2];
        }

        public void setTime(double time) {
            this.time = time;
        }

        public void setAccel(SPOutput start, SPOutput end) {
            this.accelX = new double[] {
                start.getAccelX(),
                end.getAccelX()
            };
            this.accelY = new double[] {
                start.getAccelY(),
                end.getAccelY()
            };
        }
    }
}
