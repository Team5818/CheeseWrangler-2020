/*
 * This file is part of CheeseWrangler-2020, licensed under the GNU General Public License (GPLv3).
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

package org.rivierarobotics.commands.turret;

import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.BasePIDSetPosition;
import org.rivierarobotics.subsystems.Turret;

@GenerateCreator
public class TurretSetAngle extends BasePIDSetPosition<Turret> {
    private final boolean isAbsolute;

    public TurretSetAngle(@Provided Turret turret, double angle, boolean isAbsolute) {
        super(turret, 1, angle, 2);
        this.isAbsolute = isAbsolute;
        addRequirements(turret);
    }

    @Override
    protected double getPositionTicks() {
        if (isAbsolute) {
            return subsystem.getAbsoluteAngle();
        } else {
            return subsystem.getAngle();
        }
    }

    @Override
    protected void setPositionTicks(double angle) {
        subsystem.setAngle(angle);
    }
}