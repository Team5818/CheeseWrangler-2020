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

package org.rivierarobotics.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.Subsystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import org.rivierarobotics.commands.DriveControl;
import org.rivierarobotics.util.RobotMap;

public class DriveTrain implements Subsystem {
    private final DriveTrainSide dLeft, dRight;
//    private final EncoderFollower pLeft, pRight;

    public DriveTrain() {
        this.dLeft = new DriveTrainSide(RobotMap.DriveTrain.Left.LEFT_TALON_MASTER, RobotMap.DriveTrain.Left.LEFT_SPARK_SLAVE_ONE,
                RobotMap.DriveTrain.Left.LEFT_SPARK_SLAVE_TWO, RobotMap.DriveTrain.Left.LEFT_INVERT);
        this.dRight = new DriveTrainSide(RobotMap.DriveTrain.Right.RIGHT_TALON_MASTER, RobotMap.DriveTrain.Right.RIGHT_SPARK_SLAVE_ONE,
                RobotMap.DriveTrain.Right.RIGHT_SPARK_SLAVE_TWO, RobotMap.DriveTrain.Right.RIGHT_INVERT);
        /*Waypoint[] pathfinderPoints = {
                new Waypoint(),
                new Waypoint(),
                new Waypoint()
        };
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
                0.005, 1.7, 2.0, 60);
        Trajectory trajectory = Pathfinder.generate(pathfinderPoints, config);
        TankModifier mod = new TankModifier(trajectory).modify(0.71);
        this.pLeft = new EncoderFollower(mod.getLeftTrajectory());
        this.pRight = new EncoderFollower(mod.getRightTrajectory());
        pLeft.configureEncoder(dLeft.getPositionTicks(), ());*/
        setDefaultCommand(new DriveControl(this));
    }

    public void setPower(double l, double r) {
        dLeft.setPower(l);
        dRight.setPower(r);
    }

    public double getAvgVelocity() {
        return (dLeft.getVelocity() + dRight.getVelocity()) / 2;
    }

    public DriveTrainSide getLeft() {
        return dLeft;
    }

    public DriveTrainSide getRight() {
        return dRight;
    }
}
