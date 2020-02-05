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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Inject;

public class  VisionAimHood extends CommandBase {
    private final Hood hood;
    private final DriveTrain driveTrain;
    private final Flywheel flywheel;
    private final VisionUtil vision;
    private final Turret turret;

    @Inject
    public VisionAimHood(Hood hd, DriveTrain dt, Flywheel fly, VisionUtil vision, Turret turret) {
        this.hood = hd;
        this.driveTrain = dt;
        this.flywheel = fly;
        this.vision = vision;
        this.turret = turret;
        addRequirements(hood, flywheel);
    }

    @Override
    public void execute() {
        double ty = vision.getLLValue("ty");
        double vy = 3.679;  //Vy constant
        double t = 0.375;   //time constant
        double h = 0.69;    //height of goal
        double m = 0.14;    //mass of ball
        double dist = h / Math.tan(Math.toRadians(ty)) + 0.74295;
        double tx = vision.getLLValue("tx") + Math.toRadians(turret.getAbsoluteAngle()); //returns actual tx using rotation of robot
        double txTurret = Math.atan2(dist * Math.sin(tx) + 0.1905, dist * Math.cos(tx)); //returns turret tx as it is offset from the camera.
        double vx = dist * Math.cos(txTurret) / t - driveTrain.getYVelocity(); //by splitting up our values in the x and y coordinates there has to be new velocities that go with it
        double vz = dist * Math.sin(txTurret) / t - driveTrain.getXVelocity();
        double vxz = Math.sqrt(Math.pow(vx, 2) + Math.pow(vz, 2)); // pythag for final velocity in the goal's direction
        double hoodAngle = Math.toDegrees(Math.atan2(vy - ((0.336 * vxz + 0.2) / m) * t, vxz)); //calculates hood angle with the Magnus Effect
        double flywheelVelocity = vxz / Math.cos(Math.toRadians(hoodAngle)) / 0.0005; //passes through value in ticks/100ms
        if (hoodAngle <= 40 && flywheelVelocity <= 12) {
            SmartDashboard.putNumber("hoodAngleSetpoint", hoodAngle);
            SmartDashboard.putNumber("hoodAnglePosition", hood.getPositionTicks());
            SmartDashboard.putNumber("flywheelVelocity", flywheelVelocity);
            //hood.setPosition(hoodAngle);
            //flywheel.setPositionTicks(flywheelVelocity);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
