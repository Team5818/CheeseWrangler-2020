/*
 * This file is part of PracticeBot-2020-example, licensed under the GNU General Public License (GPLv3).
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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.controller.PIDController;

public class DriveTrainSide {
    private final WPI_TalonSRX masterTalon;
    private final CANSparkMax sparkSlaveOne, sparkSlaveTwo;
    private final PIDController pidLoop;
    //TODO tune drive PID and determine ticks to inches factor
    private final double kP = 0.0005, kI = 0.0, kD = 0.0, ticksToInches = 1 / 1;

    public DriveTrainSide(int master, int slaveOne, int slaveTwo, boolean invert) {
        this.masterTalon = new WPI_TalonSRX(master);
        this.sparkSlaveOne = new CANSparkMax(slaveOne, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.sparkSlaveTwo = new CANSparkMax(slaveTwo, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.pidLoop = new PIDController(kP, kI, kD);

        masterTalon.setInverted(invert);
        sparkSlaveOne.setInverted(!invert);
        sparkSlaveTwo.setInverted(!invert);

        //TODO switch to NeutralIdleMode based mode set
        masterTalon.setNeutralMode(NeutralMode.Brake);
        sparkSlaveOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
        sparkSlaveTwo.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setPower(double pwr) {
        masterTalon.set(pwr);
        sparkSlaveOne.set(pwr);
        sparkSlaveTwo.set(pwr);
    }

    public double getPositionTicks() {
        return masterTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getPositionInches() {
        return getPositionTicks() / ticksToInches;
    }

    public void setPosition(double position) {
        pidLoop.setSetpoint(position * ticksToInches);
    }
}
