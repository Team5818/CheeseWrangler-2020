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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.rivierarobotics.util.NeutralIdleMode;

public class DriveTrainSide {
    private final WPI_TalonSRX masterTalon;
    private final CANSparkMax sparkSlaveOne, sparkSlaveTwo;
    private DriveTrain.Gear currentGear;

    //TODO transition to four-talon setup for comp bot (wait for mechanical)
    public DriveTrainSide(DriveTrainMotorIds motors, boolean invert) {
        this.masterTalon = new WPI_TalonSRX(motors.masterTalon);
        this.sparkSlaveOne = new CANSparkMax(motors.sparkSlaveOne, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.sparkSlaveTwo = new CANSparkMax(motors.sparkSlaveTwo, CANSparkMaxLowLevel.MotorType.kBrushless);

        masterTalon.configFactoryDefault();
        masterTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

        NeutralIdleMode.BRAKE.applyTo(masterTalon, sparkSlaveOne, sparkSlaveTwo);
        masterTalon.setInverted(invert);
        sparkSlaveOne.setInverted(!invert);
        sparkSlaveTwo.setInverted(!invert);
    }

    public void setPower(double pwr) {
        masterTalon.set(pwr);
        sparkSlaveOne.set(pwr);
        sparkSlaveTwo.set(pwr);
    }

    public double getPositionTicks() {
        return masterTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getVelocity() {
        return masterTalon.getSensorCollection().getQuadratureVelocity();
    }

    //TODO implement motor choosing based on gear (wait for mechanical)
    public void setGear(DriveTrain.Gear gear) {
        this.currentGear = gear;
    }

    public void setNeutralIdle(NeutralIdleMode mode) {
        mode.applyTo(masterTalon, sparkSlaveOne, sparkSlaveTwo);
    }
}
