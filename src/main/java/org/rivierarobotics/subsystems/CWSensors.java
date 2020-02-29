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

package org.rivierarobotics.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;

public class CWSensors {
    private final AnalogInput frontSensor;
    private final AnalogInput backSensor;

    public CWSensors(int front, int back) {
        this.frontSensor = new AnalogInput(front);
        this.backSensor = new AnalogInput(back);
    }

    public boolean isFrontBallPresent() {
        return (frontSensor.getValue() < 300 && frontSensor.getValue() > 1);
    }

    public double getFrontSensorValue() {
        return frontSensor.getValue();
    }


    public boolean isBackBallPresent() {
        return (backSensor.getValue() < 300 && backSensor.getValue() > 1);
    }

    public double getBackSensorValue() {
        return backSensor.getValue();
    }
}
