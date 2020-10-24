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

package org.rivierarobotics.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class VisionUtil {
    private final NetworkTable limelight;

    @Inject
    public VisionUtil() {
        NetworkTableInstance.getDefault().setServer("10.58.18.2", 5818);
        this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getLLValue(String key) {
        return getLLValue(key, -1);
    }

    public double getLLValue(String key, double def) {
        return limelight.getEntry(key).getDouble(def);
    }

    public double getActualTY(double hoodAbsPos) {
        return getLLValue("ty") + hoodAbsPos -  3;
    }

    public void setLEDState(LimelightLEDState state) {
        limelight.getEntry("ledMode").setNumber(state.ordinal());
    }
    
    public boolean hasLEDState(LimelightLEDState state) {
        return (int) limelight.getEntry("ledMode").getNumber(-1) == state.ordinal();
    }

    public void invertLedState() {
        NetworkTableEntry led = limelight.getEntry("ledMode");
        int cs = (int) led.getNumber(1.0);
        led.setNumber(cs == 1 ? 3 : 1);
    }
}
