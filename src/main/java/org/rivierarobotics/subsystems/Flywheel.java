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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Flywheel extends BasePIDSubsystem {
    private final WPI_TalonFX flywheelFalcon;

    public Flywheel(int id) {
        super(new PIDConfig(0.00075, 0.075, 0.0, 1), 600.0 / 360);
        flywheelFalcon = new WPI_TalonFX(id);
        flywheelFalcon.configFactoryDefault();
        flywheelFalcon.setInverted(false);
        flywheelFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
        flywheelFalcon.setNeutralMode(NeutralMode.Coast);

        flywheelFalcon.configNominalOutputForward(0);
        flywheelFalcon.configNominalOutputReverse(0);
        flywheelFalcon.configPeakOutputForward(1);
        flywheelFalcon.configPeakOutputReverse(-1);

        flywheelFalcon.config_kP(0, 0.35);
        flywheelFalcon.config_kI(0, 0.001);
        flywheelFalcon.config_kD(0, 5);
        flywheelFalcon.config_kF(0, 1023.0 / 20660.0);
    }

    @Override
    public double getPositionTicks() {
        return flywheelFalcon.getSelectedSensorVelocity();
    }

    @Override
    public void setPower(double pwr) {
        flywheelFalcon.set(pwr);
    }

    public void setVelocity(double vel) {
        if (vel == 0) {
            flywheelFalcon.set(TalonFXControlMode.Disabled, 0.0);
        } else {
            flywheelFalcon.set(TalonFXControlMode.Velocity, vel);
        }
    }
}
