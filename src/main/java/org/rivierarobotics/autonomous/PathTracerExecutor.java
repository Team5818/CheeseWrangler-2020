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

/**
 * Main PathTracer executor command class. Moves a robot along a path as
 * described by a passed <code>SplinePath</code> object. Contains a series of
 * waypoints interpolated by a <code>CreationMode</code> into a smooth path.
 * Contains an angular proportional loop for error correction. There is no
 * linear error correction at present.
 *
 * <p>Use <code>Pose2dPath</code> and the PathWeaver GUI to create new paths.
 * All features are supported by PathTracer. Supports field-centric and
 * robot-centric paths (given correct gyroscopic alignment and
 * drivetrain tracking). Does not use a pose estimator (e.x. Kalman
 * Filter). Calculations delegated to <code>SplinePath</code> objects.
 * Created because PathWeaver had substaintial issues running
 * on the 2020 robot.</p>
 *
 * @see SplinePath
 * @see PathConstraints
 * @see Pose2dPath
 */
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

    /**
     * Initializes command each time it is run (constructor is once globally).
     * Determines if the robot should turn to the start heading, move to a
     * start position, or stay in the same place and reset the drivetrain.
     * Also initializes logging and calculates path interpolation.
     */
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

    /**
     * Turns the robot to a heading as determined by a passed point. Given
     * directly if the tangent velocity is not precomputed (anything except
     * Quintic Hermite path trajectory generation).
     *
     * @param pt the SplinePoint to retrieve the heading from.
     */
    private void rotateToPointHeading(SplinePoint pt) {
        driveCommands.rotateTo(pt.isPrecomputedTan() ? Math.acos(pt.getTanVX()) : pt.getHeading()).schedule();
    }

    /**
     * Returns the minimum distance between two angles.
     *
     * @param a1 the first angle to compare, in degrees.
     * @param a2 the second angle to compare, in degrees.
     * @return the difference between the angles.
     */
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

    /**
     * Execute method called every 20ms while the command is running.
     * Retrieves calculations from <code>SplinePath</code> object and then
     * converts XY to LR velocities. Angular corrections added in, then set
     * to drivetrain as PIDF loops on separate sides.
     */
    @Override
    public void execute() {
        tab.setEntry("ptReset", t < 0.2);
        if (t < path.getTotalTime()) {
            // Figure out time parameter, opposite if reversed
            // Determines calculates position on path
            double timeParam = t;
            if (constraints.getReversed()) {
                timeParam = path.getTotalTime() - t;
            }
            SPOutput calc = path.calculate(timeParam);
            // Calculate angular error amount and log
            double fAng = -MathUtil.wrapToCircle(Math.toDegrees(Math.atan2(calc.getVelY(), calc.getVelX())));
            double gAng = MathUtil.wrapToCircle(gyro.getYaw());
            tab.setEntry("fAng", fAng);
            tab.setEntry("gAng", gAng);
            double angErr = angDiff(fAng, gAng);
            tab.setEntry("angErr", angErr);

            // Assorted logging functions regarding calculated path
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

            // Convert XY velocity to LR velocity, reverse if necessary
            Pair<Double> wheelSpeeds = path.getLRVel(calc);
            if (constraints.getReversed()) {
                wheelSpeeds.setA(-wheelSpeeds.getA());
                wheelSpeeds.setB(-wheelSpeeds.getB());
            }
            // Add correction based on angular error
            angErr *= ANGULAR_KP;
            if (angErr > 0) {
                wheelSpeeds.setA(wheelSpeeds.getA() + angErr);
            } else {
                wheelSpeeds.setB(wheelSpeeds.getB() - angErr);
            }
            tab.setEntry("vL", wheelSpeeds.getA());
            tab.setEntry("vR", wheelSpeeds.getB());

            // Log maximum left and right velocities
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

    /**
     * Called after the command has ended (i.e. when isFinished() returns
     * true). Resets last path heading, turns off drivetrain, and resets
     * logging outputs on the Shuffleboard.
     */
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

    /**
     * Override to end the path command.
     *
     * @return if the elapsed time is greater than the total duration of the
     *     path time as calculated by <code>SplinePath</code>.
     */
    @Override
    public boolean isFinished() {
        return t >= path.getTotalTime();
    }
}
