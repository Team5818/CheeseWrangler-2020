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

import javax.inject.Inject;
import javax.inject.Provider;

public class CheeseWheelCommands {
    private CWIncrementIndexCreator incrementIndex;
    private CWDecrementIndexCreator decrementIndex;
    private Provider<CWSetClosestIndex> setClosestIndexProvider;
    private CWSetPositionCreator setPositionCreator;
    private CWSetIndexCreator setIndexCreator;
    private WaitForBallIntakeTriggerCreator waitForBallIntakeTriggerCreator;

    @Inject
    public CheeseWheelCommands(CWIncrementIndexCreator incrementIndex,
                               CWDecrementIndexCreator decrementIndex,
                               Provider<CWSetClosestIndex> setClosestIndexProvider,
                               CWSetPositionCreator setPositionCreator,
                               WaitForBallIntakeTriggerCreator waitForBallIntakeTriggerCreator,
                               CWSetIndexCreator setIndexCreator) {
        this.incrementIndex = incrementIndex;
        this.decrementIndex = decrementIndex;
        this.setClosestIndexProvider = setClosestIndexProvider;
        this.setPositionCreator = setPositionCreator;
        this.setIndexCreator = setIndexCreator;
        this.waitForBallIntakeTriggerCreator = waitForBallIntakeTriggerCreator;
    }

    public CWIncrementIndex incrementIndex() {
        return incrementIndex.create();
    }

    public CWDecrementIndex decrementIndex() {
        return decrementIndex.create();
    }

    public CWSetClosestIndex setClosestIndex() {
        return setClosestIndexProvider.get();
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
}
