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

package org.rivierarobotics.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.rivierarobotics.subsystems.LimelightServo;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class VisionUtil {
    private final NetworkTable limelight;
    private final LimelightServo limelightServo;

    @Inject
    public VisionUtil(LimelightServo limelightServo) {
        this.limelightServo = limelightServo;
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public final double getLLValue(String key) {
        //TODO this is a bad idea, make this its own wrapper method - also key == ty probably wasn't working as intended
        if (key.equals("ty")) {
            return limelight.getEntry(key).getDouble(0) + limelightServo.getAngle();
        }
        return limelight.getEntry(key).getDouble(0);
    }

    public final void setLedState(LimelightLedState state) {
        limelight.getEntry("ledMode").setNumber(state.set);
    }

    public final void setPipMode(LimelightPIPMode mode) {
        limelight.getEntry("stream").setNumber(mode.set);
    }

    public final void invertLedState() {
        NetworkTableEntry led = limelight.getEntry("ledMode");
        double cs = (double) led.getNumber(1.0);
        led.setNumber(cs == 1 ? 3 : 1);
    }

}
