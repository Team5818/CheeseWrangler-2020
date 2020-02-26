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
import org.rivierarobotics.commands.CheeseWheelControl;

import javax.inject.Provider;

public class CheeseWheel extends BasePIDSubsystem {
    public final double indexDiff = 360.0 / 5;
    private final WPI_TalonSRX wheelTalon;
    private final DigitalInput intakeSensor;
    private final DigitalInput outputSensor;
    private final Provider<CheeseWheelControl> command;
    private final double zeroTicks = -200;
    public Mode mode = Mode.COLLECT_FRONT;
    public Mode lastMode = Mode.COLLECT_FRONT;

    public CheeseWheel(int motor, int sensorOne, int sensorTwo, Provider<CheeseWheelControl> command) {
        super(new PIDConfig(0.0012, 0.0, 0, 0.0, 15, 0.5));
        this.wheelTalon = new WPI_TalonSRX(motor);
        this.intakeSensor = new DigitalInput(sensorOne);
        this.outputSensor = new DigitalInput(sensorTwo);
        this.command = command;
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
        pidController.enableContinuousInput(0, 4095);
    }

    public boolean getIntakeSensorState() {
        return intakeSensor.get();
    }

    public boolean getOutputSensorState() {
        return outputSensor.get();
    }

    public void setMode(Mode mode) {
        if (mode == Mode.LAST) {
            this.mode = lastMode;
        } else {
            this.lastMode = this.mode;
            this.mode = mode;
        }
    }

    @Override
    public double getPositionTicks() {
        return wheelTalon.getSensorCollection().getPulseWidthPosition() % 4096;
    }

    @Override
    protected void setPower(double pwr) {
        wheelTalon.set(pwr);
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }

    @Override
    public double getPosition() {
        double position = super.getPosition();
        return position - (zeroTicks / getAnglesOrInchesToTicks());
    }

    @Override
    public void setPosition(double position) {
        position += (zeroTicks / getAnglesOrInchesToTicks());
        super.setPosition(position);
    }

    public enum Mode {
        //TODO set offsets from bottom to position
        SHOOTING(0), COLLECT_FRONT(0), COLLECT_BACK(0), CLIMB(0), LAST(0);

        public final int offset;

        Mode(int offset) {
            this.offset = offset;
        }
    }
}
