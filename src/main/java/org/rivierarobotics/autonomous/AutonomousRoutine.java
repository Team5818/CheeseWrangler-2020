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
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.DriveTrainSide;

import javax.inject.Inject;

@GenerateCreator
public class AutonomousRoutine extends CommandBase {
    private DriveTrain driveTrain;
    private Trajectory.Config configuration;
    private Trajectory trajectory;

    //TODO make this command more generic and allow any path to be passed through so that it takes as many waypoints as
    // needed (you'd pass: Waypoint... points) and waypoint objects could be created and passed. Also make an enum to
    // store those waypoint configurations so there's a selection choice for the high-level command
    // (overloaded constructor with an enum pointing to the Waypoint... points)
    Waypoint[] points = new Waypoint[] {      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
            new Waypoint(2, 2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
            new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
    };

    @Inject
    public AutonomousRoutine(@Provided DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Inject
    public AutonomousRoutine(@Provided DriveTrain driveTrain, WaypointConfigs configs) {
        this.driveTrain = driveTrain;
        buildRoutine(configs);
    }

    public void buildRoutine(WaypointConfigs configs) {
        configuration = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
        trajectory = Pathfinder.generate(WaypointConfigs.CONFIG_ONE, configuration);
    }

    public void run() {
        EncoderFollower leftFollower = new EncoderFollower(trajectory);
        EncoderFollower rightFollower = new EncoderFollower(trajectory);

        leftFollower.configureEncoder((int) driveTrain.getLeft().getPositionTicks(), 4096, 0.10414);
        rightFollower.configureEncoder((int) driveTrain.getRight().getPositionTicks(), 4096, 0.10414);

        leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);
        rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);

        driveTrain.setPower(leftFollower.calculate((int)driveTrain.getLeft().getPositionTicks()),
                rightFollower.calculate((int)driveTrain.getRight().getPositionTicks()));
    }
}
