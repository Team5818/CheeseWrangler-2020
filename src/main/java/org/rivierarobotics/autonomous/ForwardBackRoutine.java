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

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.DriveTrainSide;

//TODO make this into a Command by extending CommandBase and putting this all into the structure appropriately
//TODO add a @GenerateConstructor to this class and follow it with an entry in AutonomousCommands
public class ForwardBackRoutine {

    //TODO make this command more generic and allow any path to be passed through so that it takes as many waypoints as
    // needed (you'd pass: Waypoint... points) and waypoint objects could be created and passed. Also make an enum to
    // store those waypoint configurations so there's a selection choice for the high-level command
    // (overloaded constructor with an enum pointing to the Waypoint... points)
    Waypoint[] points = new Waypoint[] {      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
            new Waypoint(2, 2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
            new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
    };
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
    Trajectory trajectory = Pathfinder.generate(points, config);

    public void run() {
        //TODO use Dagger to inject drivetrain instead of making an object of it, use @Provided on the constructor
        DriveTrainSide leftTrain = new DriveTrainSide(new DriveTrainSide.MotorIds(4, 5, 6), true);
        DriveTrainSide rightTrain = new DriveTrainSide(new DriveTrainSide.MotorIds(1, 2, 3), false);

        EncoderFollower leftFollower = new EncoderFollower(trajectory);
        EncoderFollower rightFollower = new EncoderFollower(trajectory);

        leftFollower.configureEncoder((int) leftTrain.getPositionTicks(), 4096, 0.15);
        rightFollower.configureEncoder((int) rightTrain.getPositionTicks(), 4096, 0.15);

        //TODO fix integer divison --> 1/4 = 0
        leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 4, 0);
        rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 4, 0);

        leftTrain.setPower(leftFollower.calculate((int) leftTrain.getPositionTicks()));
        rightTrain.setPower(rightFollower.calculate((int) rightTrain.getPositionTicks()));
    }
}
