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

import org.rivierarobotics.commands.BasePIDSetPosition;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.CheeseSlot;

public class CWCycleSlot extends BasePIDSetPosition<CheeseWheel> {
    public CWCycleSlot(CheeseWheel cheeseWheel, CheeseWheel.Direction direction, CheeseWheel.AngleOffset mode, boolean requireOpen) {
        super(cheeseWheel, 10, cheeseWheel.getSlotTickPos(
            CheeseSlot.slotOfNum((cheeseWheel.getClosestSlot(mode, direction, requireOpen).ordinal()
                + (direction == CheeseWheel.Direction.BACKWARDS ? 4 : 1)) % 5), direction), 5);
    }
}
