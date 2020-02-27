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

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.Hood;

//TODO remove this once compbot is up fully
@GenerateCreator
public class HoodAlignQuadrature extends CommandBase {
    private final Hood hood;

    public HoodAlignQuadrature(@Provided Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        //TODO why do we need this if?
        if (isFinished()) {
            return;
        }
        hood.setManualPower(0.15);
    }

    @Override
    public void end(boolean interrupted) {
        hood.setManualPower(0.0);
        hood.getHoodFalcon().getSensorCollection().setIntegratedSensorPosition(0, 100);
    }

    @Override
    public boolean isFinished() {
        return hood.isAtEnd();
    }
}
