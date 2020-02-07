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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

public class CheeseWheel extends BasePIDSubsystem {
    public final double diff = 4096.0 / 5;
    private final WPI_TalonSRX wheelTalon;
    private final DigitalInput intakeSensor, outputSensor;
    //TODO change baseTicks
    public int currentIndex = 0;
    public boolean shootMode = false;
    private int baseTicks = 0, shootOffset = 0;

    public CheeseWheel(int motor, int sensorOne, int sensorTwo) {
        super(0.0, 0.0, 0.0, 1.0);
        this.wheelTalon = new WPI_TalonSRX(motor);
        this.intakeSensor = new DigitalInput(sensorOne);
        this.outputSensor = new DigitalInput(sensorTwo);
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
    }

    public boolean getIntakeSensorState() {
        return intakeSensor.get();
    }

    public boolean getOutputSensorState() {
        return outputSensor.get();
    }

    public double getIndexPosition(int index) {
        return baseTicks + ((shootMode) ? shootOffset : 0) + (index * diff);
    }

    public double getRelativeIndex() {
        return (getPositionTicks() - baseTicks - ((shootMode) ? shootOffset : 0)) / diff;
    }

    @Override
    public double getPositionTicks() {
        return wheelTalon.getSensorCollection().getPulseWidthPosition();
    }

    @Override
    protected void setPower(double pwr) {
        wheelTalon.set(pwr);
    }
}
