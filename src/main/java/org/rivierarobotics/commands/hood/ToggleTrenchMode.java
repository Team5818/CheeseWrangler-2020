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

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.turret.TurretCommands;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.HoodPosition;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Inject;

public class ToggleTrenchMode extends CommandBase {
    private final Hood hood;
    private final TurretCommands turretCommands;
    private final HoodCommands hoodCommands;
    private boolean hoodMoved = false;

    @Inject
    public ToggleTrenchMode(Hood hood, TurretCommands turretCommands, HoodCommands hoodCommands) {
        this.hood = hood;
        this.turretCommands = turretCommands;
        this.hoodCommands = hoodCommands;
    }

    @Override
    public void initialize() {
        turretCommands.setRelativeAngle(0).schedule();
        if (!hood.isTrench) {
            hood.isTrench = true;
            hoodCommands.setAngle(HoodPosition.BACK_TRENCH).schedule();
        } else {
            hoodCommands.setAngle(HoodPosition.FORWARD).schedule();
        }
        hoodMoved = true;
    }

    @Override
    public void end(boolean interrupted) {
        hood.isTrench = false;
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(hood.getAngle(), HoodPosition.BACK_TRENCH.angle, 2) && hoodMoved;
    }
}
