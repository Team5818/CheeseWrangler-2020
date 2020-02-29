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

import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.VisionTarget;

import javax.inject.Inject;

public class CheeseWheelCommands {
    private CWMoveToFreeIndexCreator moveToFreeIndex;
    private CWSetPositionCreator setPositionCreator;
    private CWSetIndexCreator setIndexCreator;
    private WaitForBallIntakeTriggerCreator waitForBallIntakeTriggerCreator;
    private final NiceShootinTexCreator niceShootinTexCreator;
    private final ShootNWedgesCreator shootNWedgesCreator;

    @Inject
    public CheeseWheelCommands(CWMoveToFreeIndexCreator moveToFreeIndex,
                               CWSetPositionCreator setPositionCreator,
                               WaitForBallIntakeTriggerCreator waitForBallIntakeTriggerCreator,
                               CWSetIndexCreator setIndexCreator,
                               NiceShootinTexCreator niceShootinTexCreator,
                               ShootNWedgesCreator shootNWedgesCreator) {
        this.moveToFreeIndex = moveToFreeIndex;
        this.setPositionCreator = setPositionCreator;
        this.setIndexCreator = setIndexCreator;
        this.waitForBallIntakeTriggerCreator = waitForBallIntakeTriggerCreator;
        this.niceShootinTexCreator = niceShootinTexCreator;
        this.shootNWedgesCreator = shootNWedgesCreator;
    }

    public CWMoveToFreeIndex moveToFreeIndex(CheeseWheel.Mode mode, CheeseWheel.Filled filled) {
        return moveToFreeIndex.create(mode, filled);
    }

    public CWSetPosition setPosition(int ticks) {
        return setPositionCreator.create(ticks);
    }

    public CWSetIndex setIndex(CheeseWheel.Mode mode, CheeseSlot index) {
        return setIndexCreator.create(mode, index);
    }

    public WaitForBallIntakeTrigger waitForBall() {
        return waitForBallIntakeTriggerCreator.create();
    }

    public NiceShootinTex niceShootinTex(int wedges) {
        return niceShootinTexCreator.create(wedges);
    }

    public ShootNWedges shootNWedges(VisionTarget target, int wedges) {
        return shootNWedgesCreator.create(target, wedges);
    }
}
