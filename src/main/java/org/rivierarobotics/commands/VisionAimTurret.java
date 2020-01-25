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
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.VisionUtil;

public class VisionAimTurret extends CommandBase {
    private final Turret turret;

    public VisionAimTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double tv = VisionUtil.getLLValue("tv");
        double tx = VisionUtil.getLLValue("tx");

        if (tv == 1) {
            double set = MathUtil.wrapToCircle(tx + turret.getPosition());
           turret.setPosition(set * turret.getAnglesOrInchesToTicks());
            SmartDashboard.putNumber("initset", set);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
