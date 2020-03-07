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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.commands.turret.TurretCommands;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.HoodPosition;

import javax.inject.Inject;

public class ToggleTrenchMode extends SequentialCommandGroup {
    private final HoodCommands hoodCommands;
    private final TurretCommands turretCommands;
    private final Hood hood;

    @Inject
    public ToggleTrenchMode(HoodCommands hoodCommands, TurretCommands turretCommands, Hood hood) {
        this.hoodCommands = hoodCommands;
        this.turretCommands = turretCommands;
        this.hood = hood;
    }

    @Override
    public void initialize() {
        if (hood.isTrench) {
            addCommands(
                turretCommands.setAngle(0),
                new WaitCommand(1.0),
                hoodCommands.setAngle(HoodPosition.BACK_TRENCH)
            );
        } else {
            addCommands(
                hoodCommands.setAngle(HoodPosition.MIDDLE)
            );
        }
        hood.isTrench = !hood.isTrench;
    }
}
