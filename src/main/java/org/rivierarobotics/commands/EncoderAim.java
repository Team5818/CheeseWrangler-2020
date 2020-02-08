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

package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.PositionTracker;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Inject;

public class EncoderAim extends InstantCommand {
    private final Hood hood;
    private final DriveTrain driveTrain;
    private final Flywheel flywheel;
    private final VisionUtil vision;
    private final Turret turret;
    private final PositionTracker tracker;

    @Inject
    public EncoderAim(Hood hood, DriveTrain dt, Flywheel flywheel, VisionUtil vision, Turret turret, PositionTracker tracker) {
        this.hood = hood;
        this.driveTrain = dt;
        this.flywheel = flywheel;
        this.vision = vision;
        this.turret = turret;
        this.tracker = tracker;
        addRequirements(hood, flywheel, turret);
    }

    @Override
    public void execute() {
        //TODO: measure values
        double fieldLength = 20; // add in actual measurements in meters
        double fieldWidthLeftWallToGoal = 10; // add in actual measurements in meters
        double pos[] = tracker.getPosition();
        double vy = 3.679;  //Vy constant
        double t = 0.375;   //time constant
        double m = 0.14;    //mass of ball
        double vx = fieldLength - pos[1] / t - driveTrain.getYVelocity(); //by splitting up our values in the x and y coordinates there has to be new velocities that go with it
        double vz = fieldWidthLeftWallToGoal - pos[0] / t - driveTrain.getXVelocity();
        double vxz = Math.sqrt(Math.pow(vx, 2) + Math.pow(vz, 2)); // pythag for final velocity in the goal's direction
        double hoodAngle = Math.toDegrees(Math.atan2(vy - ((0.336 * vxz + 0.2) / m) * t, vxz)); //calculates hood angle with the Magnus Effect
        double flywheelVelocity = vxz / Math.cos(Math.toRadians(hoodAngle)) / 0.0005; //passes through value in ticks/100ms
        double turretAngle = Math.toDegrees(Math.atan2(vz, vx)); //nice and simple angle calculation
        if (hoodAngle <= 40 && flywheelVelocity <= 12) {
            hood.setPosition(hoodAngle);
            flywheel.setPositionTicks(flywheelVelocity);
            turret.setAbsolutePosition(turretAngle);
        }


    }
}