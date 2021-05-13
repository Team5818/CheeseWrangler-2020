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

import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.MotionMagicSetPosition;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.RobotShuffleboard;

import javax.inject.Inject;

/**
 * Command to cycle the CheeseWheel one slot. Direction depends on passed
 * enum. May not be exactly one slot if AngleOffset is different from the
 * previous set. Will move with respect to the new slot setting (i.e. the
 * wheel may pass over more than one ball (or none) if going to a shooting
 * position from a collect position. Either AngleOffset value should work for
 * both collection sides, but the correct one should be used to avoid invalid
 * safety movements.
 *
 * @see CWCycleSlotInterrupt
 * @see CheeseWheel.Direction
 * @see CheeseWheel.AngleOffset
 * @see CheeseSlot.State
 */
@GenerateCreator
public class CWCycleSlot extends MotionMagicSetPosition<CheeseWheel> {
    @Inject
    public CWCycleSlot(@Provided CheeseWheel cheeseWheel, @Provided RobotShuffleboard shuffleboard,
                       CheeseWheel.Direction direction, CheeseWheel.AngleOffset mode, CheeseSlot.State requiredState,
                       double tolerance) {
        super(cheeseWheel, cheeseWheel::getPositionTicks, cheeseWheel::setPositionTicks,
            cheeseWheel.getSlotTickPos(cheeseWheel.getClosestSlot(mode, direction, requiredState), mode, direction),
                tolerance, 5, shuffleboard);
    }
}
