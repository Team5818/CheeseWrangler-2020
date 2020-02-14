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
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.Turret;

@GenerateCreator
public class TurretSetAngle extends BasePIDSetPosition<Turret> {

    private static final double zeroTicks = 1383;
    private final Turret turret;
    private double position;


    public TurretSetAngle(@Provided Turret turret, double angle) {
        super(turret, 0.3, angle);
        this.turret = turret;
    }

    @Override
    protected void setPositionTicks(double angle) {
        position = turret.getPositionTicks() + ((angle - turret.getAbsoluteAngle()) * turret.getAnglesOrInchesToTicks());
        SmartDashboard.putNumber("turretset", position);
        if (position >= zeroTicks + 150 * turret.getAnglesOrInchesToTicks() || position <= zeroTicks - 150 * turret.getAnglesOrInchesToTicks()) {
            position = position - 4096;
        }
        super.setPositionTicks(position);
    }

    @Override
    public boolean isFinished() {
        if (position < zeroTicks + 150 * turret.getAnglesOrInchesToTicks() && position > zeroTicks - 150 * turret.getAnglesOrInchesToTicks()) {
            return super.isFinished();
        } else {
            return true;
        }
    }
}
