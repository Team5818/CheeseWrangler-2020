/*
 * This file is part of Placeholder-2020, licensed under the GNU General Public License (GPLv3).
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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.util.NavXGyro;

@GenerateCreator
public class PathweaverExecutor extends CommandBase {
    private final DriveTrain driveTrain;
    private final NavXGyro gyro;
    private final RamseteController controller;
    private final Trajectory trajectory;
    private Translation2d startingPoint;
    private double startTimestamp;

    public PathweaverExecutor(@Provided DriveTrain driveTrain, Pose2dPath path) {
        this.driveTrain = driveTrain;
        this.gyro = driveTrain.getGyro();
        this.controller = new RamseteController();
        this.trajectory = generateTrajectory(path);

        addRequirements(driveTrain);
    }

    private static Trajectory generateTrajectory(Pose2dPath path) {
        TrajectoryConfig configuration = new TrajectoryConfig(1.7, 2.0);
        return TrajectoryGenerator.generateTrajectory(path.pointMap, configuration);
    }

    @Override
    public void initialize() {
        startTimestamp = Timer.getFPGATimestamp();
        startingPoint = new Translation2d(
            driveTrain.getLeft().getPosition(),
            driveTrain.getRight().getPosition()
        );
    }

    @Override
    public void execute() {
        Pose2d current = driveTrain.getPose();
        Trajectory.State goal = trajectory.sample(Timer.getFPGATimestamp() - startTimestamp);
        ChassisSpeeds adjustedSpeeds = controller.calculate(current, goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = driveTrain.getKinematics().toWheelSpeeds(adjustedSpeeds);
        driveTrain.setVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setPower(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return trajectory.getTotalTimeSeconds() >= (Timer.getFPGATimestamp() - startTimestamp);
    }
}
