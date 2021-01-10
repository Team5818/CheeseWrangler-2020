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
import org.rivierarobotics.util.Vec2D;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class SplinePath {
    public static final double MAX_POSSIBLE_VEL = 4.5; // m/s
    public static final double MAX_POSSIBLE_ACCEL = 2.0; // m/s^2
    private static final double RIO_LOOP_TIME_MS = 0.02;
    private final List<SplinePoint> points;
    private final double maxVel;
    private final double maxAccel;
    private final PathConstraints constraints;
    private final List<SPOutput> precomputed;
    private final HashMap<Double, Section> sections;
    private Pair<Double> extrema;
    private double totalTime;

    public SplinePath(List<SplinePoint> points, PathConstraints constraints) {
        this.points = points;
        this.constraints = constraints;
        this.maxVel = Math.min(constraints.getMaxVel(), MAX_POSSIBLE_VEL);
        this.maxAccel = Math.min(constraints.getMaxAccel(), MAX_POSSIBLE_ACCEL);
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
        this(points, PathConstraints.create());
    }

    public void recalculatePath() {
        totalTime = 0;
        sections.clear();
        precomputed.clear();
        Section sc;
        double scTime;
        for (int i = 1; i < points.size(); i++) {
            sc = new Section(points.get(i - 1), points.get(i), i - 1);
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
        double[] te = {
            1, t, t * t, t * t * t, t * t * t * t, t * t * t * t * t
        };
        double time = timeCorrected ? section.time : 1;
        // fixedTheta = Cubic Hermite Splines, else Catmull-Rom Splines
        if (constraints.getFixedTheta()) {
            double[] h = {
                1 - 10 * te[3] + 15 * te[4] - 6 * te[5],
                t - 6 * te[3] + 8 * te[4] - 3 * te[5],
                0.5 * te[2] - 1.5 * te[3] + 1.5 * te[4] - 0.5 * te[5],
                0.5 * te[3] - te[4] + 0.5 * te[5],
                -4 * te[3] + 7 * te[4] - 3 * te[5],
                10 * te[3] - 15 * te[4] + 6 * te[5]
            };
            double[] hd = {
                -30 * te[4] + 60 * te[3] - 30 * te[2],
                -15 * te[4] + 32 * te[3] - 18 * te[2] + 1,
                -2.5 * te[4] + 6 * te[3] - 4.5 * te[2] + t,
                2.5 * te[4] - 4 * te[3] + 1.5 * te[2],
                -15 * te[4] + 28 * te[3] - 12 * te[2],
                30 * te[4] - 60 * te[3] + 30 * te[2]
            };
            double[] hdd = {
                -120 * te[3] + 180 * te[2] - 60 * t,
                -60 * te[3] + 96 * te[2] - 36 * t,
                -10 * te[3] + 18 * te[2] - 9 * t + 1,
                10 * te[3] - 12 * te[2] + 3 * t,
                -60 * te[3] + 84 * te[2] - 24 * t,
                120 * te[3] - 180 * te[2] + 60 * t
            };

            return new SPOutput(
                h[0] * section.p0.getX() + h[1] * section.tanVX[0] + h[2] * section.accelX[0] + h[3] * section.accelX[1] + h[4] * section.tanVX[1] + h[5] * section.p1.getX(),
                h[0] * section.p0.getY() + h[1] * section.tanVY[0] + h[2] * section.accelY[0] + h[3] * section.accelY[1] + h[4] * section.tanVY[1] + h[5] * section.p1.getY(),
                (hd[0] * section.p0.getX() + hd[1] * section.tanVX[0] + hd[2] * section.accelX[0] + hd[3] * section.accelX[1] + hd[4] * section.tanVX[1] + hd[5] * section.p1.getX()) / time,
                (hd[0] * section.p0.getY() + hd[1] * section.tanVY[0] + hd[2] * section.accelY[0] + hd[3] * section.accelY[1] + hd[4] * section.tanVY[1] + hd[5] * section.p1.getY()) / time,
                (hdd[0] * section.p0.getX() + hdd[1] * section.tanVX[0] + hdd[2] * section.accelX[0] + hdd[3] * section.accelX[1] + hdd[4] * section.tanVX[1] + hdd[5] * section.p1.getX()) / time,
                (hdd[0] * section.p0.getY() + hdd[1] * section.tanVY[0] + hdd[2] * section.accelY[0] + hdd[3] * section.accelY[1] + hdd[4] * section.tanVY[1] + hdd[5] * section.p1.getY()) / time
            );
        } else {
            // Scaling factor of 0.01m (1cm) offset for endpoints (required to not be control sequence t=[0,1])
            double linearSlope = ((section.p1.getY() - section.p0.getY()) / (section.p1.getX() - section.p0.getX())) * 0.01;
            Vec2D[] sps = {
                section.p0idx == 0 ? new Vec2D(section.p0.getX() - linearSlope, section.p0.getY() - linearSlope) : points.get(section.p0idx - 1),
                points.get(section.p0idx),
                points.get(section.p0idx + 1),
                (points.size() <= section.p0idx + 2) ? new Vec2D(section.p1.getX() + linearSlope, section.p1.getY() + linearSlope) : points.get(section.p0idx + 2)
            };
            double t01 = Math.sqrt(sps[0].dist(sps[1]));
            double t12 = Math.sqrt(sps[1].dist(sps[2]));
            double t23 = Math.sqrt(sps[2].dist(sps[3]));

            Vec2D m1 = sps[2].addVec(sps[1].negate()).addVec(sps[1].addVec(sps[0].negate()).multNum(1 / t01).addVec(sps[2].addVec(sps[0].negate()).multNum(1 / (t01 + t12)).negate()).multNum(t12));
            Vec2D m2 = sps[2].addVec(sps[1].negate()).addVec(sps[3].addVec(sps[2].negate()).multNum(1 / t23).addVec(sps[3].addVec(sps[1].negate()).multNum(1 / (t12 + t23)).negate()).multNum(t12));
            Vec2D[] sec = {
                sps[1].addVec(sps[2].negate()).multNum(2).addVec(m1).addVec(m2),
                sps[1].addVec(sps[2].negate()).multNum(-3).addVec(m1.negate()).addVec(m1.negate()).addVec(m2.negate()),
                m1,
                sps[1]
            };
            Vec2D pos = sec[0].multNum(te[3]).addVec(sec[1].multNum(te[2])).addVec(sec[2].multNum(t)).addVec(sec[3]);
            Vec2D vel = sec[0].multNum(3 * te[2]).addVec(sec[1].multNum(2 * t)).addVec(sec[2]).multNum(1 / time);
            Vec2D accel = sec[0].multNum(6 * t).addVec(sec[1].multNum(2)).multNum(1 / time);

            return new SPOutput(pos.getX(), pos.getY(), vel.getX(), vel.getY(), accel.getX(), accel.getY());
        }
    }

    // Assumes the robot is moving with the correct trajectory
    public Pair<Double> getLRVel(SPOutput calc) {
        return getLRVel(calc.getVelX(), calc.getVelY(), Math.atan2(calc.getVelY(), calc.getVelX()));
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
            // tempAccel = new Pair<>(tempOut.getAccelX(), tempOut.getAccelY());
            if (Math.abs(tempAccel.getA()) > maxObservedAccel) {
                maxObservedAccel = Math.abs(tempAccel.getA());
            }
            if (Math.abs(tempAccel.getB()) > maxObservedAccel) {
                maxObservedAccel = Math.abs(tempAccel.getB());
            }
        }
        // accel *2 b/c of derivative 1/2
        return Math.max(maxObservedVel / maxVel, 2 * maxObservedAccel / maxAccel);
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

    public double getTotalDistance() {
        double dist = 0;
        for (Section s : sections.values()) {
            dist += s.dist;
        }
        return dist;
    }

    public PathConstraints getConstraints() {
        return constraints;
    }

    public static class Section {
        private final SplinePoint p0;
        private final SplinePoint p1;
        private final int p0idx;
        private final double[] tanVX;
        private final double[] tanVY;
        private final double dist;
        private double[] accelX;
        private double[] accelY;
        private double time;

        public Section(SplinePoint p0, SplinePoint p1, int p0idx) {
            this.p0 = p0;
            this.p1 = p1;
            this.p0idx = p0idx;
            this.dist = p0.dist(p1);
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
