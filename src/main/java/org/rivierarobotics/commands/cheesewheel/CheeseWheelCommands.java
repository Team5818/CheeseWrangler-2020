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

import javax.inject.Inject;

public class CheeseWheelCommands {
    private CWSetPositionCreator setPositionCreator;
    private CWSetIndexCreator setIndexCreator;
    private final ShootNWedgesCreator shootNWedgesCreator;
    private final CWMoveToNextIndexCreator moveToNextIndexCreator;
    private All5ShootCreator all5ShootCreator;
    private CWMoveToNextIndexInterruptCreator moveToNextInterruptCreator;

    @Inject
    public CheeseWheelCommands(CWSetPositionCreator setPositionCreator,
                               CWSetIndexCreator setIndexCreator,
                               ShootNWedgesCreator shootNWedgesCreator,
                               CWMoveToNextIndexCreator moveToNextIndexCreator,
                               All5ShootCreator all5ShootCreator,
                               CWMoveToNextIndexInterruptCreator moveToNextInterruptCreator) {
        this.setPositionCreator = setPositionCreator;
        this.setIndexCreator = setIndexCreator;
        this.shootNWedgesCreator = shootNWedgesCreator;
        this.moveToNextIndexCreator = moveToNextIndexCreator;
        this.all5ShootCreator = all5ShootCreator;
        this.moveToNextInterruptCreator = moveToNextInterruptCreator;
    }

    public CWMoveToNextIndexInterrupt moveToNextIndexCancel(int direction, CheeseWheel.AngleOffset mode) {
        return moveToNextInterruptCreator.create(direction, mode);
    }

    public CWSetPosition setPosition(int ticks) {
        return setPositionCreator.create(ticks);
    }

    public CWSetIndex setIndex(CheeseWheel.AngleOffset mode, int index, int direction) {
        return setIndexCreator.create(mode, index, direction);
    }

    public ShootNWedges shootNWedges(int wedges) {
        return shootNWedgesCreator.create(wedges);
    }

    public CWMoveToNextIndex moveToNextIndex(int direction, CheeseWheel.AngleOffset mode) {
        return moveToNextIndexCreator.create(direction, mode);
    }

    public All5Shoot all5Shoot() {
        return all5ShootCreator.create();
    }
}
