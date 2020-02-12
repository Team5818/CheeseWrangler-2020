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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.VisionUtil;

@GenerateCreator
public class VisionAimHood extends CommandBase {
    private final Hood hood;
    private final DriveTrain driveTrain;
    private final Flywheel flywheel;
    private final VisionUtil vision;
    private final Turret turret;
    private final double extraDistance;
    private final double height;

    //TODO remove parameters you don't want to set with this command and addRequirements() the ones you want to move with this command
    public VisionAimHood(@Provided Hood hd, @Provided DriveTrain dt, @Provided Flywheel fly, @Provided VisionUtil vision, @Provided Turret turret, double extraDistance, double height) {
        this.hood = hd;
        this.driveTrain = dt;
        this.flywheel = fly;
        this.vision = vision;
        this.turret = turret;
        this.extraDistance = extraDistance;
        this.height = height;
        addRequirements(hood, flywheel);
    }

    @Override
    public void execute() {
        double ty = vision.getLLValue("ty");
        double vy = 3.679;  //Vy constant
        double t = 0.375;   //time constant
        double h = height;    //height of goal (0.69)
        double m = 0.14;    //mass of ball
        double dist = h / Math.tan(Math.toRadians(ty));
        SmartDashboard.putNumber("distancetotarget", dist);
        double tx = vision.getLLValue("tx") + turret.getAbsoluteAngle();
        tx = Math.toRadians(tx);
        SmartDashboard.putNumber("modifiedtx", Math.toDegrees(tx));
        double txTurret = Math.atan2(dist * Math.sin(tx) - 0.21, dist * Math.cos(tx)); //returns turret tx as it is offset from the camera.
        SmartDashboard.putNumber("turrettx", Math.toDegrees(txTurret));
        double vx = (dist * Math.cos(txTurret) + extraDistance) / t - driveTrain.getYVelocity(); //by splitting up our values in the x and y coordinates there has to be new velocities that go with it
        double vz = dist * Math.sin(txTurret) / t - driveTrain.getXVelocity();
        double vxz = Math.sqrt(Math.pow(vx, 2) + Math.pow(vz, 2)); // pythag for final velocity in the goal's direction
        double hoodAngle = Math.toDegrees(Math.atan2(vy, vxz)); //calculates hood angle with the Magnus Effect
        double flywheelVelocity = vxz / Math.cos(Math.toRadians(hoodAngle)); //VALUE IN METERS / SECOND
        double encoderVelocity = ((flywheelVelocity - 0.86) / .003) * (1 / 600.0) * 4.4 * 12;
        SmartDashboard.putNumber("hoodAngleSetpoint", hoodAngle);
        SmartDashboard.putNumber("EncoderVelocity", encoderVelocity);
        SmartDashboard.putNumber("flywheelVelocity", flywheelVelocity);
        if (hoodAngle <= 40 && flywheelVelocity <= 12) {
            hood.setAbsolutePosition(hoodAngle + 2);
            flywheel.setPositionTicks(encoderVelocity + 10);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
