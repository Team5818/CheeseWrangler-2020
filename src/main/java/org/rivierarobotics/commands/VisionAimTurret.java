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
import org.rivierarobotics.util.MathUtil;
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
        double tv = vision.getLLValue("tv");
        double tx = vision.getLLValue("tx");
        double ty = vision.getLLValue("ty");
        double dtVel = driveTrain.getAvgVelocity();

        if (tv == 1) {
            double t = 0.375;   //time constant
            double h = 0.69;    //height of goal
            double dist = h / Math.tan(Math.toRadians(ty)); //change 0's to VXrobot and VYrobot once available
            double turretAngle = -Math.atan2(0, dist / t);
            double set = MathUtil.wrapToCircle(tx - turretAngle) + MathUtil.wrapToCircle(turret.getPosition());
            turret.setPositionTicks(set * turret.getAnglesOrInchesToTicks());
            SmartDashboard.putNumber("initset", set);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
