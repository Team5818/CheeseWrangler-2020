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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;

@GenerateCreator
public class PathweaverExecutor extends CommandBase {
    private final DriveTrain driveTrain;
    private final RamseteController controller;
    private final Trajectory trajectory;
    private final RobotShuffleboardTab tab;
    private double startTimestamp;

    public PathweaverExecutor(@Provided DriveTrain driveTrain, @Provided RobotShuffleboard shuffleboard, Pose2dPath path) {
        this.driveTrain = driveTrain;
        this.controller = new RamseteController();
        this.trajectory = generateTrajectory(path);
        this.tab = shuffleboard.getTab("Pathweaver");
        addRequirements(driveTrain);
    }

    private Trajectory generateTrajectory(Pose2dPath path) {
        final var traj = path.getTrajectory();
        return traj.relativeTo(traj.getInitialPose());
    }

    @Override
    public void initialize() {
        startTimestamp = Timer.getFPGATimestamp();
        driveTrain.resetEncoder();
    }

    @Override
    public void execute() {
        Pose2d current = driveTrain.getPose();
        Trajectory.State goal = trajectory.sample(Timer.getFPGATimestamp() - startTimestamp);
        tab.setEntry("CurrentPose", current.toString());
        tab.setEntry("GoalPose", goal.toString());
        ChassisSpeeds adjustedSpeeds = controller.calculate(current, goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = driveTrain.getKinematics().toWheelSpeeds(adjustedSpeeds);
        tab.setEntry("Left Vel Set", wheelSpeeds.leftMetersPerSecond);
        tab.setEntry("Right Vel Set", wheelSpeeds.rightMetersPerSecond);
        driveTrain.setVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setVelocity(0.0, 0.0);
        driveTrain.setPower(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return trajectory.getTotalTimeSeconds() <= (Timer.getFPGATimestamp() - startTimestamp);
    }
}
