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

import org.rivierarobotics.util.LimelightLedState;
import org.rivierarobotics.util.VisionTarget;

import javax.inject.Inject;

public class VisionCommands {
    private VisionAimHoodCreator visionAimHoodCreator;
    private VisionAimTurretCreator visionAimTurretCreator;
    private LimelightLedToggleCreator limelightLedToggleCreator;
    private VisionAimCreator visionAimCreator;

    @Inject
    public VisionCommands(VisionAimHoodCreator visionAimHoodCreator,
                          VisionAimTurretCreator visionAimTurretCreator,
                          LimelightLedToggleCreator limelightLedToggleCreator,
                          VisionAimCreator visionAimCreator) {
        this.visionAimHoodCreator = visionAimHoodCreator;
        this.visionAimCreator = visionAimCreator;
        this.visionAimTurretCreator = visionAimTurretCreator;
        this.limelightLedToggleCreator = limelightLedToggleCreator;
    }

    public VisionAimHood autoAimHood(double extraDistance, double height) {
        return visionAimHoodCreator.create(extraDistance, height);
    }

    public VisionAimTurret autoAimTurret(double extraDistance, double height) {
        return visionAimTurretCreator.create(extraDistance, height);
    }

    public VisionAim visionAim(VisionTarget target) {
        return visionAimCreator.create(target);
    }

    public LimelightLedSetState limelightSetState(LimelightLedState state) {
        return limelightLedToggleCreator.create(state);
    }
}
