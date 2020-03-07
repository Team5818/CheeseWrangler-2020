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

package org.rivierarobotics.commands.cheesewheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.CheeseSlot;

@GenerateCreator
public class CWMoveToNextIndex extends CommandBase {
    private final CheeseWheel cheeseWheel;
    private CheeseSlot currentSlot;
    private final int direction;

    public CWMoveToNextIndex(@Provided CheeseWheel cheeseWheel, int direction) {
        this.cheeseWheel = cheeseWheel;
        this.direction = direction;
        addRequirements(cheeseWheel);
    }

    @Override
    public void initialize() {
        currentSlot = cheeseWheel
            .getClosestSlot(CheeseWheel.Mode.COLLECT_FRONT, CheeseWheel.Filled.DONT_CARE, 0)
            .next(direction);
        cheeseWheel.setPositionTicks(currentSlot.getModePosition(CheeseWheel.Mode.COLLECT_FRONT));
    }

    @Override
    public boolean isFinished() {
        return cheeseWheel.getPidController().atSetpoint();
    }
}
