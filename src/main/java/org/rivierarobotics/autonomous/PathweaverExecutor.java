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

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.PIDConfig;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@GenerateCreator
public class PathweaverExecutor extends CommandBase {
    private static final double MAX_VEL = 4.0; // Maximum velocity (in m/s)
    private static final double MAX_ACCEL = 2.0; // Maximum acceleration (in m/s)
    private static final double MAX_DRAW_VOLTAGE = 10.0; // Maximum motor draw voltage (< robot)
    // Specifies ks, kv, and ka constants - found via robot characterization
    //TODO do robot-characterization for MOTOR_FF and tune voltage PID
    private static final SimpleMotorFeedforward MOTOR_FF = new SimpleMotorFeedforward(0, 0, 0);
    private static final PIDConfig PID_CONFIG = new PIDConfig(0.05, 0, 0);
    private static final Twist2d ADD_180_FLIP = new Twist2d(0, 0, Math.toRadians(180));

    private final DriveTrain driveTrain;
    private final RamseteCommand command;
    private final Trajectory trajectory;
    private final RobotShuffleboardTab tab;

    public PathweaverExecutor(@Provided DriveTrain driveTrain, @Provided RobotShuffleboard shuffleboard, Pose2dPath path, boolean isAbsolute, boolean flip) {
        this.driveTrain = driveTrain;
        this.trajectory = generateTrajectory(path, isAbsolute, flip);
        this.command = createCommand();
        this.tab = shuffleboard.getTab("Pathweaver");
    }

    // Included for compatibility w/ preexisting autos
    public PathweaverExecutor(@Provided DriveTrain driveTrain, @Provided RobotShuffleboard shuffleboard, Pose2dPath path) {
        this(driveTrain, shuffleboard, path, true, false);
    }

    private Trajectory generateTrajectory(Pose2dPath path, boolean isAbsolute, boolean flip) {
        Trajectory pathTraj = path.getTrajectory();
        if (flip) {
            pathTraj = new Trajectory(trajectory.getStates().stream().map(state ->
                    new Trajectory.State(trajectory.getTotalTimeSeconds() - state.timeSeconds,
                            -state.velocityMetersPerSecond,
                            -state.accelerationMetersPerSecondSq,
                            state.poseMeters.exp(ADD_180_FLIP),
                            state.curvatureRadPerMeter)
            ).collect(Collectors.toList()));
        }
        List<Pose2d> pathPoses = new ArrayList<>();
        for (Trajectory.State state : pathTraj.getStates()) {
            pathPoses.add(state.poseMeters);
        }
        Trajectory out = TrajectoryGenerator.generateTrajectory(pathPoses,
            new TrajectoryConfig(MAX_VEL, MAX_ACCEL)
                .setKinematics(driveTrain.getKinematics())
                .addConstraint(new DifferentialDriveVoltageConstraint(
                    MOTOR_FF, driveTrain.getKinematics(), MAX_DRAW_VOLTAGE)));
        return isAbsolute ? out : out.relativeTo(driveTrain.getPose());
    }

    private RamseteCommand createCommand() {
        return new RamseteCommand(
            trajectory,
            driveTrain::getPose,
            new RamseteController(),
            MOTOR_FF,
            driveTrain.getKinematics(),
            () -> new DifferentialDriveWheelSpeeds(
                driveTrain.getLeft().getVelocity(),
                driveTrain.getRight().getVelocity()
            ),
            new PIDController(PID_CONFIG.getP(), PID_CONFIG.getI(), PID_CONFIG.getD()),
            new PIDController(PID_CONFIG.getP(), PID_CONFIG.getI(), PID_CONFIG.getD()),
            this::loggedVoltageOut,
            driveTrain
        );
    }

    public void loggedVoltageOut(double l, double r) {
        tab.setEntry("VoltageSetLeft", l);
        tab.setEntry("VoltageSetRight", r);
        tab.setEntry("BatVoltage", RobotController.getBatteryVoltage());
        driveTrain.setVoltage(l, r);
    }

    @Override
    public void initialize() {
        tab.setEntry("TotalTime", trajectory.getTotalTimeSeconds());
        command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setVoltage(0, 0);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
