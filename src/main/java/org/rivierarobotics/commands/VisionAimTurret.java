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
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.VisionUtil;

@GenerateCreator
public class VisionAimTurret extends CommandBase {
    private final Turret turret;
    private final DriveTrain driveTrain;
    private final VisionUtil vision;
    private final double extraDistance;
    private final double height;

    //TODO remove parameters you don't want to set with this command and addRequirements() the ones you want to move with this command
    public VisionAimTurret(@Provided Turret turret, @Provided DriveTrain driveTrain, @Provided VisionUtil vision, double extraDistance, double height) {
        this.turret = turret;
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.height = height;
        this.extraDistance = extraDistance;
        addRequirements(turret, driveTrain);
    }

    @Override
    public void execute() {
        double ty = vision.getLLValue("ty");
        double t = 0.375;   //time constant
        double h = height;    //height of goal (0.69 for practice atm)
        double dist = h / Math.tan(Math.toRadians(ty));
        SmartDashboard.putNumber("distancetotarget", dist);
        double tx = vision.getLLValue("tx") + turret.getAbsoluteAngle();
        tx = Math.toRadians(tx);
        SmartDashboard.putNumber("modifiedtx", Math.toDegrees(tx));
        double txTurret = Math.atan2(dist * Math.sin(tx) - 0.21, dist * Math.cos(tx)); //returns turret tx as it is offset from the camera.
        SmartDashboard.putNumber("turrettx", Math.toDegrees(txTurret));
        double vx = (dist * Math.cos(txTurret) + extraDistance) / t - driveTrain.getYVelocity(); //by splitting up our values in the x and y coordinates there has to be new velocities that go with it
        double vz = dist * Math.sin(txTurret) / t - driveTrain.getXVelocity();
        double turretAngle = Math.toDegrees(Math.atan2(vz, vx)); //nice and simple angle calculation
        double tv = vision.getLLValue("tv");
        if (tv == 1) {
            turret.setAbsolutePosition(turretAngle);
            SmartDashboard.putNumber("setABS", turretAngle);
        }
    }

}
