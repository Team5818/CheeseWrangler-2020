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

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class SplinePath {
    public static final double MAX_POSSIBLE_VEL = 4.5; // m/s
    public static final double MAX_POSSIBLE_ACCEL = 2.0; // m/s^2
    public static final double RIO_LOOP_TIME_MS = 0.02;
    private final LinkedList<SplinePoint> points;
    private final double maxVel;
    private final double maxAccel;
    private final PathConstraints constraints;
    private final LinkedList<SPOutput> precomputed;
    private final LinkedHashMap<Double, Section> sections;
    private Pair<Vec2D> nftCtrlPoints;
    private Pair<Double> extrema;
    private double totalTime;
    private Double lastOmega;

    public SplinePath(List<SplinePoint> points, PathConstraints constraints) {
        this.points = new LinkedList<>(points);
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

        if (constraints.getCreationMode() == PathConstraints.CreationMode.CATMULL_ROM) {
            // Scaling factor of 0.1m (10cm) offset for Catmull-Rom endpoints
            // (required to not be control sequence t=[0,1])
            nftCtrlPoints = points.size() >= 2 ? new Pair<>(
                    extrapolateEndpoint(0, 1, 0, true),
                    extrapolateEndpoint(points.size() - 2, points.size() - 1,
                            points.size() - 1, false)
            ) : new Pair<>(Vec2D.createBlank(), Vec2D.createBlank());
        }
    }

    public SplinePath(List<SplinePoint> points) {
        this(points, PathConstraints.create());
    }

    private Vec2D extrapolateEndpoint(int p0idx, int p1idx, int focusIdx, boolean invert) {
        final double m = 0.1;
        double dy = points.get(p1idx).getY() - points.get(p0idx).getY();
        double dx = points.get(p1idx).getX() - points.get(p0idx).getX();
        return points.get(focusIdx).addVec(new Vec2D(invert ? -m : m * dx, invert ? -m : m * dy));
    }

    public void recalculatePath() {
        System.out.println();
        totalTime = 0;
        sections.clear();
        precomputed.clear();
        Section sc;
        double scTime;
        for (int i = 1; i < points.size(); i++) {
            sc = new Section(points.get(i - 1), points.get(i), i - 1);
            scTime = Math.max(0.05, timeAtLimited(sc, true));
            sc.setTime(scTime);
            sc.setAccel(calculate(sc, RIO_LOOP_TIME_MS, false), calculate(sc, 1 - RIO_LOOP_TIME_MS, false));
            sections.put(totalTime, sc);
            totalTime += scTime;
        }
        resetOmega();
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
        switch (constraints.getCreationMode()) {
            case QUINTIC_HERMITE:
                double[] qh = {
                    1 - 10 * te[3] + 15 * te[4] - 6 * te[5],
                    t - 6 * te[3] + 8 * te[4] - 3 * te[5],
                    0.5 * te[2] - 1.5 * te[3] + 1.5 * te[4] - 0.5 * te[5],
                    0.5 * te[3] - te[4] + 0.5 * te[5],
                    -4 * te[3] + 7 * te[4] - 3 * te[5],
                    10 * te[3] - 15 * te[4] + 6 * te[5]
                };
                double[] qhd = {
                    -30 * te[2] + 60 * te[3] - 30 * te[4],
                    1 - 18  * te[2] + 32 * te[3] - 15 * te[4],
                    t - 4.5 * te[2] + 6 * te[3] - 2.5 * te[4],
                    1.5 * te[2] - 4 * te[3] + 2.5 * te[4],
                    -12 * te[2] + 28 * te[3] - 15 * te[4],
                    30 * te[2] - 60 * te[3] + 30 * te[4]
                };
                double[] qhdd = {
                    -120 * te[3] + 180 * te[2] - 60 * t,
                    -60 * te[3] + 96 * te[2] - 36 * t,
                    -10 * te[3] + 18 * te[2] - 9 * t + 1,
                    10 * te[3] - 12 * te[2] + 3 * t,
                    -60 * te[3] + 84 * te[2] - 24 * t,
                    120 * te[3] - 180 * te[2] + 60 * t
                };

                return new SPOutput(
                    qh[0] * section.p0.getX() + qh[1] * section.tanVX[0] + qh[2] * section.accelX[0] + qh[3] * section.accelX[1] + qh[4] * section.tanVX[1] + qh[5] * section.p1.getX(),
                    qh[0] * section.p0.getY() + qh[1] * section.tanVY[0] + qh[2] * section.accelY[0] + qh[3] * section.accelY[1] + qh[4] * section.tanVY[1] + qh[5] * section.p1.getY(),
                    (qhd[0] * section.p0.getX() + qhd[1] * section.tanVX[0] + qhd[2] * section.accelX[0] + qhd[3] * section.accelX[1] + qhd[4] * section.tanVX[1] + qhd[5] * section.p1.getX()) / time,
                    (qhd[0] * section.p0.getY() + qhd[1] * section.tanVY[0] + qhd[2] * section.accelY[0] + qhd[3] * section.accelY[1] + qhd[4] * section.tanVY[1] + qhd[5] * section.p1.getY()) / time,
                    (qhdd[0] * section.p0.getX() + qhdd[1] * section.tanVX[0] + qhdd[2] * section.accelX[0] + qhdd[3] * section.accelX[1] + qhdd[4] * section.tanVX[1] + qhdd[5] * section.p1.getX()) / time,
                    (qhdd[0] * section.p0.getY() + qhdd[1] * section.tanVY[0] + qhdd[2] * section.accelY[0] + qhdd[3] * section.accelY[1] + qhdd[4] * section.tanVY[1] + qhdd[5] * section.p1.getY()) / time
                );
            case CUBIC_HERMITE:
                double[] ch = {
                    1 - 3 * te[2] + 2 * te[3],
                    t - 2 * te[2] + te[3],
                    -te[2] + te[3],
                    3 * te[2] - 2 * te[3]
                };
                double[] chd = {
                    -6 * t + 6 * te[2],
                    1 - 4 * t + 3 * te[2],
                    -2 * t + 3 * te[2],
                    6 * t - 6 * te[2]
                };
                double[] chdd = {
                    -6 + 12 * t,
                    -4 + 6 * t,
                    -2 + 6 * t,
                    6 - 12 * t
                };

                return new SPOutput(
                        ch[0] * section.p0.getX() + ch[1] * section.tanVX[0] + ch[2] * section.tanVX[1] + ch[3] * section.p1.getX(),
                        ch[0] * section.p0.getY() + ch[1] * section.tanVY[0] + ch[2] * section.tanVY[1] + ch[3] * section.p1.getY(),
                        (chd[0] * section.p0.getX() + chd[1] * section.tanVX[0] + chd[2] * section.tanVX[1] + chd[3] * section.p1.getX()) / time,
                        (chd[0] * section.p0.getY() + chd[1] * section.tanVY[0] + chd[2] * section.tanVY[1] + chd[3] * section.p1.getY()) / time,
                        (chdd[0] * section.p0.getX() + chdd[1] * section.tanVX[0] + chdd[2] * section.tanVX[1] + chdd[3] * section.p1.getX()) / time,
                        (chdd[0] * section.p0.getY() + chdd[1] * section.tanVY[0] + chdd[2] * section.tanVY[1] + chdd[3] * section.p1.getY()) / time
                );
            case CATMULL_ROM:
                Vec2D[] sps = {
                    section.p0idx == 0 ? nftCtrlPoints.getA() : points.get(section.p0idx - 1),
                    points.get(section.p0idx),
                    points.get(section.p0idx + 1),
                    (points.size() <= section.p0idx + 2) ? nftCtrlPoints.getB() : points.get(section.p0idx + 2)
                };
                double t01 = Math.pow(sps[0].dist(sps[1]), constraints.getCrKnotParam());
                double t12 = Math.pow(sps[1].dist(sps[2]), constraints.getCrKnotParam());
                double t23 = Math.pow(sps[2].dist(sps[3]), constraints.getCrKnotParam());

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
            default:
                throw new IllegalArgumentException("No valid creation mode found");
        }
    }

    public static double angleDiff(double a1, double a2) {
        double diff = (a2 - a1 + Math.PI) % (2 * Math.PI) - Math.PI;
        return diff < -Math.PI ? diff + (2 * Math.PI) : diff;
    }

    // Assumes the robot is moving with the correct trajectory
    public Pair<Double> getLRVel(SPOutput lastCalc, SPOutput instCalc) {
        double omega = Math.atan2(instCalc.getPosY() - lastCalc.getPosY(), instCalc.getPosX() - lastCalc.getPosX());
        double tempOmega = omega;
        if (lastOmega == null) {
            lastOmega = tempOmega;
        }
        omega = angleDiff(omega, lastOmega);
        lastOmega = tempOmega;

        double angluarVel = omega / RIO_LOOP_TIME_MS * DriveTrain.getTrackwidth() / 2;
        if (constraints.getReversed()) {
            angluarVel *= -1;
        }

        double linearVel = Math.sqrt((instCalc.getVelX() * instCalc.getVelX()) + (instCalc.getVelY() * instCalc.getVelY()));
        if (constraints.getStraight()) {
            angluarVel = 0;
        }
        return new Pair<>(
                linearVel - angluarVel,
                linearVel + angluarVel
        );
    }

    public double timeAtLimited(Section section, boolean addPrecomputed) {
        Pair<Double> lastTempVel = new Pair<>(0.0, 0.0);
        Pair<Double> instTempVel;
        SPOutput lastTempOut = null;
        SPOutput instTempOut;
        double maxObservedVel = 0;
        for (double t = 0; t < 1; t += RIO_LOOP_TIME_MS) {
            instTempOut = calculate(section, t, false);
            if (addPrecomputed) {
                precomputed.add(instTempOut);
            }

            if (lastTempOut == null) {
                lastTempOut = instTempOut;
            }

            instTempVel = getLRVel(lastTempOut, instTempOut);
            if (Math.abs(instTempVel.getA()) > maxObservedVel) {
                maxObservedVel = Math.abs(instTempVel.getA());
            }
            if (Math.abs(instTempVel.getB()) > maxObservedVel) {
                maxObservedVel = Math.abs(instTempVel.getB());
            }
        }
        return maxObservedVel / maxVel;
    }

    public void resetOmega() {
        lastOmega = null;
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

        @Override
        public String toString() {
            return "Section{"
                    + "p0=" + p0
                    + ", p1=" + p1
                    + ", p0idx=" + p0idx
                    + ", tanVX=" + Arrays.toString(tanVX)
                    + ", tanVY=" + Arrays.toString(tanVY)
                    + ", dist=" + dist
                    + ", accelX=" + Arrays.toString(accelX)
                    + ", accelY=" + Arrays.toString(accelY)
                    + ", time=" + time
                    + '}';
        }
    }
}
