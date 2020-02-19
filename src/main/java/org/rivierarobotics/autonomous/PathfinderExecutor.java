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

import edu.wpi.first.wpilibj2.command.CommandBase;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.DriveTrainSide;
import org.rivierarobotics.subsystems.EncoderType;
import org.rivierarobotics.util.NavXGyro;

@GenerateCreator
public class PathfinderExecutor extends CommandBase {
    private DriveTrain driveTrain;
    private NavXGyro gyro;
    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;

    public PathfinderExecutor(@Provided DriveTrain driveTrain, WaypointPath path) {
        this.driveTrain = driveTrain;
        this.gyro = driveTrain.getGyro();
        this.leftFollower = driveTrain.getLeft().getEncoderFollower();
        this.rightFollower = driveTrain.getRight().getEncoderFollower();

        Trajectory.Config configuration = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
        Trajectory trajectory = Pathfinder.generate(path.pointMap, configuration);

        leftFollower.configureEncoder((int) driveTrain.getLeft().getPositionTicks(), EncoderType.REV_THROUGH_BORE.ticksPerRev, 0.10414);
        rightFollower.configureEncoder((int) driveTrain.getRight().getPositionTicks(), EncoderType.REV_THROUGH_BORE.ticksPerRev, 0.10414);

        leftFollower.setTrajectory(trajectory);
        rightFollower.setTrajectory(trajectory);
    }

    @Override
    public void execute() {
        driveTrain.setPower(
            determineSideSpeed(leftFollower, true),
            determineSideSpeed(rightFollower, false)
        );
    }

    public double determineSideSpeed(EncoderFollower follower, boolean isLeft) {
        DriveTrainSide dts = isLeft ? driveTrain.getLeft() : driveTrain.getRight();
        double base = follower.calculate((int) dts.getPositionTicks());

        // This allows the angle difference to respect 'wrapping', where 360 and 0 are the same value
        double angleDifference = Pathfinder.boundHalfDegrees(Pathfinder.r2d(follower.getHeading()) - gyro.getYaw());
        angleDifference = angleDifference % 360.0;
        if (Math.abs(angleDifference) > 180.0) {
            angleDifference = (angleDifference > 0) ? angleDifference - 360 : angleDifference + 360;
        }

        //TODO revise this to reflect our robot
        double turn = 0.8 * (-1.0 / 80.0) * angleDifference;
        return base + (turn * (isLeft ? 1 : -1));
    }

    @Override
    public boolean isFinished() {
        return leftFollower.isFinished() && rightFollower.isFinished();
    }
}
