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

public class CWSetClosestHalfIndex extends BasePIDSetPosition<CheeseWheel> {
    @Inject
    public CWSetClosestHalfIndex(CheeseWheel cheeseWheel) {
        super(cheeseWheel, 0.05, 0);
    }

    @Override
    protected void setSetPosition(double position) {
        positionTicks = Math.floor(subsystem.getRelativeIndex()) + 0.5;
    }

    @Override
    protected double getPositionTicks() {
        return subsystem.getRelativeIndex();
    }

    @Override
    protected void setPositionTicks(double position) {
        //subsystem.getSetIndex(position);
    }
}
