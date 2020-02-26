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

import javax.inject.Inject;
import javax.inject.Provider;

public class CheeseWheelCommands {
    private CWIncrementIndexCreator incrementIndexCreator;
    private CWDecrementIndexCreator decrementIndexCreator;
    private Provider<CWSetClosestHalfIndex> setClosestHalfIndexProvider;
    private Provider<CWShootAll> shootAllProvider;
    private Provider<CWShootIndividual> shootIndividualProvider;
    private CWSetPositionCreator setPositionCreator;
    private CWSetModeCreator setModeCreator;
    private CWInvertModeCreator invertModeCreator;
    private CWAutoCollectCreator autoCollectCreator;

    @Inject
    public CheeseWheelCommands(CWIncrementIndexCreator incrementIndexCreator,
                               CWDecrementIndexCreator decrementIndexCreator,
                               Provider<CWSetClosestHalfIndex> setClosestHalfIndexProvider,
                               Provider<CWShootAll> shootAllProvider,
                               Provider<CWShootIndividual> shootIndividualProvider,
                               CWSetPositionCreator setPositionCreator,
                               CWSetModeCreator setModeCreator,
                               CWInvertModeCreator invertModeCreator,
                               CWAutoCollectCreator autoCollectCreator) {
        this.incrementIndexCreator = incrementIndexCreator;
        this.decrementIndexCreator = decrementIndexCreator;
        this.setClosestHalfIndexProvider = setClosestHalfIndexProvider;
        this.shootAllProvider = shootAllProvider;
        this.shootIndividualProvider = shootIndividualProvider;
        this.setPositionCreator = setPositionCreator;
        this.setModeCreator = setModeCreator;
        this.invertModeCreator = invertModeCreator;
        this.autoCollectCreator = autoCollectCreator;
    }

    public CWIncrementIndex incrementIndex() {
        return incrementIndexCreator.create();
    }

    public CWDecrementIndex decrementIndex() {
        return decrementIndexCreator.create();
    }

    public CWSetClosestHalfIndex setClosestHalfIndex() {
        return setClosestHalfIndexProvider.get();
    }

    public CWShootAll shootAll() {
        return shootAllProvider.get();
    }

    public CWShootIndividual shootNext() {
        return shootIndividualProvider.get();
    }

    public CWSetPosition setPosition(int ticks) {
        return setPositionCreator.create(ticks);
    }

    public CWSetMode setMode(CheeseWheel.Mode mode) {
        return setModeCreator.create(mode);
    }

    public CWInvertMode invertMode() {
        return invertModeCreator.create();
    }

    public CWAutoCollect autoCollect(boolean front) {
        return autoCollectCreator.create(front);
    }
}
