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
import javax.inject.Singleton;

@Singleton
public class CheeseWheel extends BasePIDSubsystem {
    public final double diff = 4096.0 / 5;
    private final WPI_TalonSRX wheelTalon;
    private final DigitalInput intakeSensor;
    private final DigitalInput outputSensor;
    private final Provider<CheeseWheelControl> command;
    private final double zeroTicks = -200;
    private int current;
    private double rots = 0;
    public int currentIndex = 0;
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
        rots = getRotations();
    }

    public boolean getIntakeSensorState() {
        return intakeSensor.get();
    }

    public boolean getOutputSensorState() {
        return outputSensor.get();
    }

    public double getRotations() {
        return getPositionTicks() % 4096;
    }

    public void setMode(Mode mode) {
        if (mode == Mode.LAST) {
            this.mode = lastMode;
        } else {
            this.lastMode = this.mode;
            this.mode = mode;
        }
    }

    public int getIndexPosition(int index) {
        current = index;
        double indexPos = zeroTicks + (index * diff);
        if( Math.abs(indexPos + diff) > zeroTicks + 3800 || Math.abs(indexPos + diff) < zeroTicks + 4200 ) {
            if(current - index < 0) {
                rots++;
            } else {
                rots--;
            }
        }
        if (indexPos != 0) {
            return (int) (indexPos + (rots * 4096));
        } else {
            return (int) indexPos;
        }
    }

    public double getRelativeIndex() {
        //TODO implement rotations
        return (getPositionTicks() - mode.offset) / diff;
    }

    @Override
    public double getPositionTicks() {
        return wheelTalon.getSensorCollection().getPulseWidthPosition();
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

    public void setIndexPosition(double position) {
        setPositionTicks(position);
    }

    @Override
    public void setPositionTicks(double position) {
        rots = getRotations();
        super.setPositionTicks(position);
    }

    public enum Mode {
        //TODO set offsets from bottom to position
        SHOOTING(0), COLLECT_FRONT(0), COLLECT_BACK(0), CLIMB(0), LAST(0);

        public final int offset;

        Mode(int offset) { this.offset = offset;
        }
    }
}
