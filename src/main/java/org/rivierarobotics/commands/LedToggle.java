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

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.util.VisionUtil;

public class LedToggle extends InstantCommand {

    private int ledState;

    public LedToggle() {
        ledState = -1;
    }

    public LedToggle(boolean ledState) {
        if (ledState)
            this.ledState = 3;
        else
            this.ledState = 1;
    }

    @Override
    public void execute() {
        if (ledState >= 0) {
            VisionUtil.LIMELIGHT.getEntry("ledMode").setNumber(ledState);
        } else if(VisionUtil.LIMELIGHT.getEntry("ledMode").getDouble(2.) == (3.)) {
            VisionUtil.LIMELIGHT.getEntry("ledMode").setNumber(1);
        } else if(VisionUtil.LIMELIGHT.getEntry("ledMode").getDouble(2.) == (1.)) {
            VisionUtil.LIMELIGHT.getEntry("ledMode").setNumber(3);
        }
    }
}
