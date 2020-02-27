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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.commands.CheeseWheelControl;
import org.rivierarobotics.util.CWSensors;
import org.rivierarobotics.util.CheeseSlots;

import javax.inject.Provider;

public class CheeseWheel extends BasePIDSubsystem {
    public final double indexDiff = 360.0 / 5;
    private final WPI_TalonSRX wheelTalon;
    private final Provider<CheeseWheelControl> command;
    private final double zeroTicks = -200;
    private final CWSensors sensors;
    public Mode mode = Mode.COLLECT_FRONT;
    public Mode lastMode = Mode.COLLECT_FRONT;

    public CheeseWheel(int motor, CWSensors sensors, Provider<CheeseWheelControl> command) {
        super(new PIDConfig(0.0012, 0.0, 0, 0.0, 15, 0.5));
        this.wheelTalon = new WPI_TalonSRX(motor);
        this.command = command;
        this.sensors = sensors;
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
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
        return Math.abs((getPositionTicks() - zeroTicks) * (1 / getAnglesOrInchesToTicks()));
    }

    @Override
    public void setPosition(double position) {
        SmartDashboard.putNumber("posi", position);
        setPositionTicks(position);

    }

    @Override
    public double getPositionTicks() {
        return wheelTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getSetIndex(double index) {
        double angle = index * 360.0 / 5 + getPosition();
        angle = angle % 360;
        SmartDashboard.putNumber("addangle", angle);
        if (angle < 180) {
            return (getPositionTicks() - angle * getAnglesOrInchesToTicks());
        } else {
            return (getPositionTicks() + (360 - angle * getAnglesOrInchesToTicks()));
        }
    }

    public int getIndex() {
        int min = 360;
        double minAngle = 360;
        SmartDashboard.putNumber("angleff", getPosition() % 360);
        for (int i = 0; i < 5; i++) {
            if (minAngle > Math.abs(getPosition() % 360 - (i * indexDiff))) {
                minAngle = Math.abs(getPosition() % 360 - (i * indexDiff));
                min = i;
            }
        }
        return min;
    }

    public int getClosestIndex(boolean lookForFilled) {
        return CheeseSlots.getClosestIndex(mode, getPosition(), lookForFilled);
    }

    public CWSensors getSensors() {
        return sensors;
    }

    public enum Mode {
        COLLECT_FRONT, COLLECT_BACK, SHOOTING, LAST;
    }
}
