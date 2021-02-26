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

package org.rivierarobotics.commands.climb;

import org.rivierarobotics.subsystems.Climb;

import javax.inject.Inject;

public class ClimbCommands {
    private final ClimbSetPositionCreator climbSetPositionCreator;
    private final ClimbSetPowerCreator climbSetPowerCreator;
    private final ClimbSetZeroCreator climbSetZeroCreator;
    private final HookSetPositionCreator hookSetPositionCreator;

    @Inject
    public ClimbCommands(ClimbSetPositionCreator climbSetPositionCreator,
                         ClimbSetPowerCreator climbSetPowerCreator,
                         ClimbSetZeroCreator climbSetZeroCreator,
                         HookSetPositionCreator hookSetPositionCreator) {
        this.climbSetPositionCreator = climbSetPositionCreator;
        this.climbSetZeroCreator = climbSetZeroCreator;
        this.climbSetPowerCreator = climbSetPowerCreator;
        this.hookSetPositionCreator = hookSetPositionCreator;
    }

    public ClimbSetPosition setClimbPosition(double position) {
        return climbSetPositionCreator.create(position);
    }

    public ClimbSetPosition setClimbPosition(Climb.Position position) {
        return climbSetPositionCreator.create(position.getTicks());
    }

    public ClimbSetZero resetEncoder() {
        return climbSetZeroCreator.create();
    }

    public ClimbSetPower setPower(double power) {
        return climbSetPowerCreator.create(power);
    }

    public HookSetPosition setHookPosition(double position) {
        return hookSetPositionCreator.create(position);
    }
}
