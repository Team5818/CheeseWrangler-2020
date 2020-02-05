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

@GenerateCreator
public class DriveTrainRoutine extends CommandBase {
    private DriveTrain driveTrain;
    private Trajectory.Config configuration;
    private Trajectory trajectory;

    //TODO note: @Provided & @GenerateCreator are better than @Inject here because we want set different configs
    public DriveTrainRoutine(@Provided DriveTrain driveTrain, WaypointConfig config) {
        this.driveTrain = driveTrain;
        buildRoutine(config);
    }

    //TODO note: an enum can store a value in a field, for example .waypoints -- an enum cannot take the place of a different type
    public void buildRoutine(WaypointConfig config) {
        configuration = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
        trajectory = Pathfinder.generate(config.waypoints, configuration);
    }

    //TODO make this into execute() with @Override and pull the EncoderFollowers out to fields.
    // Make an isFinished method that returns true if both followers are finished (follower.isFinished())
    public void run() {
        EncoderFollower leftFollower = new EncoderFollower(trajectory);
        EncoderFollower rightFollower = new EncoderFollower(trajectory);

        leftFollower.configureEncoder((int) driveTrain.getLeft().getPositionTicks(), 4096, 0.10414);
        rightFollower.configureEncoder((int) driveTrain.getRight().getPositionTicks(), 4096, 0.10414);

        leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);
        rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);

        driveTrain.setPower(leftFollower.calculate((int) driveTrain.getLeft().getPositionTicks()),
                rightFollower.calculate((int) driveTrain.getRight().getPositionTicks()));
    }
}
