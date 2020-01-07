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

/**
 * One side of the DriveTrain, either right or left
 * Comprised of talon/spark/spark, not a subsystem but in essence 1/2 of one
 */
public class DriveTrainSide {
    private final WPI_TalonSRX masterTalon;
    private final CANSparkMax sparkSlaveOne, sparkSlaveTwo;
    private final PIDController pidLoop;
    private final double kP = 0.0005, kI = 0.0, kD = 0.0, ticksToInches = 1 / 1;

    /**
     * Constructs a DriveTrainSide complete with talon/spark/spark motors
     * inverted and with brake mode set, and PID loop applied.
     * @param master the primary talon ID
     * @param slaveOne the first secondary spark ID
     * @param slaveTwo the second secondary spark ID
     * @param invert the status of whether or not the motors should be inverted.
     */
    public DriveTrainSide(int master, int slaveOne, int slaveTwo, boolean invert) {
        this.masterTalon = new WPI_TalonSRX(master);
        this.sparkSlaveOne = new CANSparkMax(slaveOne, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.sparkSlaveTwo = new CANSparkMax(slaveTwo, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.pidLoop = new PIDController(kP, kI, kD);

        masterTalon.setInverted(invert);
        sparkSlaveOne.setInverted(!invert);
        sparkSlaveTwo.setInverted(!invert);

        masterTalon.setNeutralMode(NeutralMode.Brake);
        sparkSlaveOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
        sparkSlaveTwo.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    /**
     * Sets power to the motors directly from input unless the PID target is reached
     * @param pwr the power to set, from -1 to 1, that goes directly to the motors
     */
    public void setPower(double pwr) {
        if(!pidLoop.atSetpoint()) {
            pwr = pidLoop.calculate(getPositionTicks());
        }
        masterTalon.set(pwr);
        sparkSlaveOne.set(pwr);
        sparkSlaveTwo.set(pwr);
    }

    /**
     * Gets the positional data from the absolute encoder of the talon
     * @return the absolute position of the DriveTrainSide in ticks/encoder units
     */
    public double getPositionTicks() {
        return masterTalon.getSensorCollection().getPulseWidthPosition();
    }

    /**
     * More concisely usable to set position than getPositionTicks(), same function
     * @return the absolute position of the DriveTrainSide in inches
     */
    public double getPositionInches() {
        return getPositionTicks() / ticksToInches;
    }

    /**
     * Sets a setpoint for the PID that will be acted on with calculate()
     * The PID, if properly tuned, will go to the position in a calm and controlled manner
     * @param position the absolute position for the robot to go to via PID
     */
    public void setPosition(double position) {
        pidLoop.setSetpoint(position * ticksToInches);
    }
}
