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

package org.rivierarobotics.commands.hood;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.turret.TurretCommands;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.HoodPosition;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Inject;

public class ToggleTrenchMode extends SequentialCommandGroup {
    private final Turret turret;
    private final Hood hood;
    private final TurretCommands turretCommands;
    private final HoodCommands hoodCommands;

    @Inject
    public ToggleTrenchMode(Turret turret, Hood hood, TurretCommands turretCommands, HoodCommands hoodCommands) {
        this.turret = turret;
        this.hood = hood;
        this.turretCommands = turretCommands;
        this.hoodCommands = hoodCommands;
        addRequirements(turret, hood);
    }

    @Override
    public void initialize() {
        turret.setAngle(0);
        hood.setAbsoluteAngle(HoodPosition.FORWARD.angle);
    }

    @Override
    public void execute() {
        MathUtil.isWithinTolerance(turret.getAbsoluteAngle(), HoodPosition.FORWARD.ticks, 20);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
