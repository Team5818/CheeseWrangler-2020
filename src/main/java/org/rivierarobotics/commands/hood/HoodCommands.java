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

package org.rivierarobotics.commands.hood;

import javax.inject.Inject;

public class HoodCommands {
    private final HoodSetAngleCreator hoodSetAngleCreator;
    private final HoodAngleAdjustCreator hoodAngleAdjustCreator;

    @Inject
    public HoodCommands(HoodSetAngleCreator hoodSetAngleCreator,
                        HoodAngleAdjustCreator hoodAngleAdjustCreator) {
        this.hoodSetAngleCreator = hoodSetAngleCreator;
        this.hoodAngleAdjustCreator = hoodAngleAdjustCreator;
    }

    public HoodSetAngle setAngle(double angle) {
        return hoodSetAngleCreator.create(angle);
    }

    public HoodAngleAdjust adjustAutoAngle(double angle) {
        return hoodAngleAdjustCreator.create(angle);
    }
}
