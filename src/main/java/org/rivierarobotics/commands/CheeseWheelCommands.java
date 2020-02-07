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

import javax.inject.Inject;
import javax.inject.Provider;

public class CheeseWheelCommands {
    private Provider<CWIncrementIndex> advanceIndexProvider;
    private Provider<CWSetClosestHalfIndex> setClosestHalfIndexProvider;
    private Provider<CWShootAll> shootAllProvider;
    private Provider<CWShootIndividual> shootIndividualProvider;
    private CWSetPositionCreator setPositionCreator;
    private CWSetShootModeCreator setShootModeCreator;

    @Inject
    public CheeseWheelCommands(Provider<CWIncrementIndex> advanceIndexProvider,
                               Provider<CWSetClosestHalfIndex> setClosestHalfIndexProvider,
                               Provider<CWShootAll> shootAllProvider,
                               Provider<CWShootIndividual> shootIndividualProvider,
                               CWSetPositionCreator setPositionCreator,
                               CWSetShootModeCreator setShootModeCreator) {
        this.advanceIndexProvider = advanceIndexProvider;
        this.setClosestHalfIndexProvider = setClosestHalfIndexProvider;
        this.shootAllProvider = shootAllProvider;
        this.shootIndividualProvider = shootIndividualProvider;
        this.setPositionCreator = setPositionCreator;
        this.setShootModeCreator = setShootModeCreator;
    }

    public CWIncrementIndex incrementIndex() {
        return advanceIndexProvider.get();
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

    public CWSetShootMode setShootMode(boolean shootMode) {
        return setShootModeCreator.create(shootMode);
    }
}
