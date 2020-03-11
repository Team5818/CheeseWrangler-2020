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

package org.rivierarobotics.autonomous;

import org.rivierarobotics.autonomous.basic.ForwardAuto;
import org.rivierarobotics.autonomous.basic.ForwardAutoCreator;
import org.rivierarobotics.autonomous.basic.ShootAndDrive;
import org.rivierarobotics.autonomous.basic.ShootAndDriveCreator;

import javax.inject.Inject;

public class AutonomousCommands {
    private PathweaverExecutorCreator pathweaverExecutorCreator;
    private ForwardAutoCreator forwardAutoCreator;
    private ShootAndDriveCreator shootAndDriveCreator;

    @Inject
    public AutonomousCommands(PathweaverExecutorCreator pathweaverExecutorCreator,
                              ForwardAutoCreator forwardAutoCreator,
                              ShootAndDriveCreator shootAndDriveCreator) {
        this.pathweaverExecutorCreator = pathweaverExecutorCreator;
        this.forwardAutoCreator = forwardAutoCreator;
        this.shootAndDriveCreator = shootAndDriveCreator;
    }

    public PathweaverExecutor pathweaver(Pose2dPath path) {
        return pathweaverExecutorCreator.create(path);
    }

    public ForwardAuto forwardAuto(boolean useVisionAim) {
        return forwardAutoCreator.create(useVisionAim);
    }

    public ShootAndDrive shootAndDrive() {
        return shootAndDriveCreator.create();
    }

}
