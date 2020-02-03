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
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Inject;

public class VisionAimTurret extends CommandBase {
    private final Turret turret;
    private final DriveTrain driveTrain;
    private final VisionUtil vision;

    @Inject
    public VisionAimTurret(Turret turret, DriveTrain dt, VisionUtil vision) {
        this.turret = turret;
        this.driveTrain = dt;
        this.vision = vision;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double ty = vision.getLLValue("ty");
        double t = 0.375;   //time constant
        double h = 0.69;    //height of goal
        double dist = h / Math.tan(Math.toRadians(ty));
        double vx = dist/t - driveTrain.getYVelocity();
        double vz = driveTrain.getXVelocity();
        double tx = Math.toRadians(vision.getLLValue("tx"));
        double turretAngle = Math.toDegrees(Math.atan2((dist*Math.tan(tx))-0.1905,dist)); //gets actual tx because camera is offset.
        double tv = vision.getLLValue("tv");
        double offset = Math.toDegrees(Math.atan2(vz,vx));
        if (tv == 1) {
            turret.setAbsolutePosition(turretAngle-offset);
            SmartDashboard.putNumber("setABS", turretAngle);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
