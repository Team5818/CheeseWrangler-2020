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

package org.rivierarobotics.commands.drive;

import org.rivierarobotics.util.NeutralIdleMode;

import javax.inject.Inject;

public class DriveCommands {
    private DriveSetNeutralIdleCreator driveSetNeutralIdleCreator;
    private DriveDistanceCreator driveDistanceCreator;

    @Inject
    public DriveCommands(DriveSetNeutralIdleCreator driveSetNeutralIdleCreator,
                         DriveDistanceCreator driveDistanceCreator) {
        this.driveDistanceCreator = driveDistanceCreator;
        this.driveSetNeutralIdleCreator = driveSetNeutralIdleCreator;
    }

    public DriveDistance driveDistance(double finalDistance, double power) {
        return driveDistanceCreator.create(finalDistance, power);
    }

    public DriveSetNeutralIdle setNeutralIdle(NeutralIdleMode mode) {
        return driveSetNeutralIdleCreator.create(mode);
    }
}