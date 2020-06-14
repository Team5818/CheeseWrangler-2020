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

import org.rivierarobotics.commands.shooting.All5Shoot;
import org.rivierarobotics.commands.shooting.All5ShootCreator;
import org.rivierarobotics.commands.shooting.ShootNWedges;
import org.rivierarobotics.commands.shooting.ShootNWedgesCreator;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.CheeseSlot;

import javax.inject.Inject;

public class CheeseWheelCommands {
    private final CWSetPositionCreator setPositionCreator;
    private final CWCycleSlotCreator cycleSlotCreator;
    private final ShootNWedgesCreator shootNWedgesCreator;
    private final CWMoveSlotToIndexCreator moveSlotToIndexCreator;
    private final All5ShootCreator all5ShootCreator;

    @Inject
    public CheeseWheelCommands(CWSetPositionCreator setPositionCreator,
                               CWCycleSlotCreator cycleSlotCreator,
                               ShootNWedgesCreator shootNWedgesCreator,
                               CWMoveSlotToIndexCreator moveSlotToIndexCreator,
                               All5ShootCreator all5ShootCreator) {
        this.setPositionCreator = setPositionCreator;
        this.cycleSlotCreator = cycleSlotCreator;
        this.shootNWedgesCreator = shootNWedgesCreator;
        this.moveSlotToIndexCreator = moveSlotToIndexCreator;
        this.all5ShootCreator = all5ShootCreator;
    }

    public CWCycleSlot cycleSlot(CheeseWheel.Direction direction, CheeseWheel.AngleOffset mode, boolean requireOpen) {
        return cycleSlotCreator.create(direction, mode, requireOpen);
    }

    public CWSetPosition setPosition(int ticks) {
        return setPositionCreator.create(ticks);
    }

    public CWMoveSlotToIndex moveSlotToIndex(CheeseWheel.AngleOffset mode, CheeseSlot slot, CheeseWheel.Direction direction) {
        return moveSlotToIndexCreator.create(mode, slot, direction);
    }

    public ShootNWedges shootNWedges(int wedges) {
        return shootNWedgesCreator.create(wedges);
    }

    public All5Shoot all5Shoot() {
        return all5ShootCreator.create();
    }
}
