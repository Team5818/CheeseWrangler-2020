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
import edu.wpi.first.wpilibj.DigitalInput;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class CWSensors {
    private final AnalogInput intakeSensor;
    private final AnalogInput outputSensor;

    @Inject
    public CWSensors(int intake, int output) {
        this.intakeSensor = new AnalogInput(intake);
        this.outputSensor = new AnalogInput(output);
    }

    public boolean getIntakeSensorStatus() {
        return (intakeSensor.getValue() < 200 && intakeSensor.getValue() > 100);
    }

    public double getIntakeSensorValue() {
        return intakeSensor.getValue();
    }


    public boolean getOutputSensorStatus() {
        return (outputSensor.getValue() < 200 && outputSensor.getValue() > 100);
    }

    public double getOutputSensorValue() {
        return outputSensor.getValue();
    }
}
