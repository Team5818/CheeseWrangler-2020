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

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.Pair;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;

@GenerateCreator
public class PathTracerExecutor extends CommandBase {
    private static final String PATH_TRACER = "PathTracer";
    private static final double ANGULAR_KP = 0.05;
    private final DriveTrain driveTrain;
    private final DriveCommands driveCommands;
    private final NavXGyro gyro;
    private final RobotShuffleboardTab tab;
    private final SplinePath path;
    private final PathConstraints constraints;
    private double gyroOffset;
    private double t;
    private double pXAccum;
    private double pYAccum;

    public PathTracerExecutor(@Provided DriveTrain driveTrain, @Provided DriveCommands driveCommands,
                              @Provided NavXGyro gyro, @Provided RobotShuffleboard shuffleboard,
                              SplinePath path) {
        this.driveTrain = driveTrain;
        this.driveCommands = driveCommands;
        this.gyro = gyro;
        this.tab = shuffleboard.getTab(PATH_TRACER);
        this.path = path;
        this.constraints = path.getConstraints();
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        gyro.resetGyro();
        gyroOffset = 0;
        pXAccum = 0;
        pYAccum = 0;
        t = 0;
        tab.setEntry("ptReset", true);
        if (constraints.getAbsPos()) {
            path.addPathPoint(0, new SplinePoint(driveTrain.getPose()));
            path.recalculatePath();
        } else if (constraints.getAbsHeading()) {
            rotateToPointHeading(path.getPathPoints().get(0));
        } else {
            double firstAngle = MathUtil.wrapToCircle(path.getPathPoints().get(0).getHeading());
            gyroOffset = MathUtil.wrapToCircle(gyro.getYaw())
                - MathUtil.wrapToCircle(path.getPathPoints().get(0).getHeading());
            tab.setEntry("firstAngle", firstAngle);
        }
        if (constraints.getReversed()) {
            gyroOffset = MathUtil.wrapToCircle(gyroOffset + 180);
        }
        path.recalculatePath();
        tab.setEntry("gyroOffset", gyroOffset);
        tab.setEntry("totalTime", path.getTotalTime());
        tab.setEntry("totalDist", path.getTotalDistance());
        tab.setEntry("vLMax", 0);
        tab.setEntry("vRMax", 0);
    }

    private void rotateToPointHeading(SplinePoint pt) {
        driveCommands.rotateTo(pt.isPrecomputedTan() ? Math.acos(pt.getTanVX()) : pt.getHeading()).schedule();
    }

    private static double angDiff(double a1, double a2) {
        a1 = MathUtil.wrapToCircle(a1);
        a2 = MathUtil.wrapToCircle(a2);
        if (a1 - a2 < -180) {
            a2 -= 360;
        } else if (a1 - a2 > 180) {
            a2 += 360;
        }
        return a1 - a2;
    }

    @Override
    public void execute() {
        tab.setEntry("ptReset", t < 0.2);
        if (t < path.getTotalTime()) {
            double timeParam = t;
            if (constraints.getReversed()) {
                timeParam = path.getTotalTime() - t;
            }
            SPOutput calc = path.calculate(timeParam);
            double fAng = -MathUtil.wrapToCircle(Math.toDegrees(Math.atan2(calc.getVelY(), calc.getVelX())));
            double gAng = MathUtil.wrapToCircle(gyro.getYaw());
            tab.setEntry("fAng", fAng);
            tab.setEntry("gAng", gAng);
            double angErr = angDiff(fAng, gAng);
            tab.setEntry("angErr", angErr);

            tab.setEntry("currTime", t);
            tab.setEntry("pX", calc.getPosX());
            tab.setEntry("pY", calc.getPosY());
            tab.setEntry("vX", calc.getVelX());
            tab.setEntry("vY", calc.getVelY());
            tab.setEntry("aX", calc.getAccelX());
            tab.setEntry("aY", calc.getAccelY());

            pXAccum += calc.getVelX() * SplinePath.RIO_LOOP_TIME_MS;
            pYAccum += calc.getVelY() * SplinePath.RIO_LOOP_TIME_MS;
            tab.setEntry("pXA", pXAccum);
            tab.setEntry("pYA", pYAccum);

            Pair<Double> wheelSpeeds = path.getLRVel(calc);
            if (constraints.getReversed()) {
                wheelSpeeds.setA(-wheelSpeeds.getA());
                wheelSpeeds.setB(-wheelSpeeds.getB());
            }
            angErr *= ANGULAR_KP;
            if (angErr > 0) {
                wheelSpeeds.setA(wheelSpeeds.getA() + angErr);
            } else {
                wheelSpeeds.setB(wheelSpeeds.getB() - angErr);
            }
            tab.setEntry("vL", wheelSpeeds.getA());
            tab.setEntry("vR", wheelSpeeds.getB());

            double vLMax = tab.getEntry("vLMax").getDouble(0);
            double vRMax = tab.getEntry("vRMax").getDouble(0);
            if (Math.abs(vLMax) < Math.abs(wheelSpeeds.getA())) {
                tab.setEntry("vLMax", wheelSpeeds.getA());
            }
            if (Math.abs(vRMax) < Math.abs(wheelSpeeds.getB())) {
                tab.setEntry("vRMax", wheelSpeeds.getB());
            }
            driveTrain.setVelocity(wheelSpeeds.getA(), wheelSpeeds.getB());
        }
        t += SplinePath.RIO_LOOP_TIME_MS;
    }

    @Override
    public void end(boolean interrupted) {
        path.resetOmega();
        driveTrain.setVelocity(0);
        driveTrain.setPower(0, 0);
        tab.setEntry("vX", 0);
        tab.setEntry("vY", 0);
        tab.setEntry("aX", 0);
        tab.setEntry("aY", 0);
        tab.setEntry("vL", 0);
        tab.setEntry("vR", 0);
    }

    @Override
    public boolean isFinished() {
        return t >= path.getTotalTime();
    }
}
