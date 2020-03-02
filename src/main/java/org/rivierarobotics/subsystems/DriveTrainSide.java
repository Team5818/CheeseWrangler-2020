/*
 * This file is part of CheeseWrangler-2020, licensed under the GNU General Public License (GPLv3).
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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import org.rivierarobotics.util.NeutralIdleMode;

public class DriveTrainSide {
    private final WPI_TalonSRX masterTalon;
    private final CANSparkMax sparkSlaveOne;
    private final CANSparkMax sparkSlaveTwo;
    private final boolean invert;
    private DriveTrain.Gear currentGear;
    private PIDController pidController;
    private boolean pidEnabled = false;
    //TODO find actual ticks to inches
    private double initialPosition;
    private final double ticksPerInch = 12720.0 / 72;

    //TODO remove when new drivetrain is implemented
    public DriveTrainSide(MotorIds motors, boolean invert) {
        this.masterTalon = new WPI_TalonSRX(motors.masterTalon);
        this.sparkSlaveOne = new CANSparkMax(motors.sparkSlaveOne, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.sparkSlaveTwo = new CANSparkMax(motors.sparkSlaveTwo, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.pidController = new PIDController(0.000002, 0.0, 0.0);
        this.invert = invert;

        masterTalon.configFactoryDefault();
        masterTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        NeutralIdleMode.BRAKE.applyTo(masterTalon, sparkSlaveOne, sparkSlaveTwo);
        masterTalon.setInverted(invert);
        sparkSlaveOne.setInverted(!invert);
        sparkSlaveTwo.setInverted(!invert);

        initialPosition = getPositionTicks();
    }

    public void setPower(double pwr) {
        pidEnabled = false;
        masterTalon.set(pwr);
        sparkSlaveOne.set(pwr);
        sparkSlaveTwo.set(pwr);
    }

    public double getPositionTicks() {
        int pos = masterTalon.getSensorCollection().getQuadraturePosition();
        if (invert) {
            pos *= -1;
        }
        return pos;
    }

    public int getVelocity() {
        int vel = masterTalon.getSensorCollection().getQuadratureVelocity();
        if (invert) {
            vel *= -1;
        }
        return vel;
    }

    public void setGear(DriveTrain.Gear gear) {
        this.currentGear = gear;
    }

    public void setNeutralIdle(NeutralIdleMode mode) {
        mode.applyTo(masterTalon, sparkSlaveOne, sparkSlaveTwo);
    }

    public void setVelocity(double vel) {
        double rawVel = Units.metersToInches(vel) * 10 * ticksPerInch;
        pidEnabled = true;
        pidController.setSetpoint(rawVel);
    }

    public void setPidPower() {
        if (pidEnabled) {
            double power = pidController.calculate(getVelocity());
            masterTalon.set(power);
            sparkSlaveOne.set(power);
            sparkSlaveTwo.set(power);
        }
    }

    public double getPosition() {
        return getPositionTicks() / ticksPerInch;
    }

    public double getOffsetPosition() {
        return (getPositionTicks() - initialPosition) / ticksPerInch;
    }

    public void resetPose() {
        initialPosition = getPositionTicks();
    }

    public void resetEncoder() {
        masterTalon.getSensorCollection().setQuadraturePosition(0, 100);
    }

    public static class MotorIds {
        public final int masterTalon;
        public final int sparkSlaveOne;
        public final int sparkSlaveTwo;

        public MotorIds(int masterTalon, int sparkSlaveOne, int sparkSlaveTwo) {
            this.masterTalon = masterTalon;
            this.sparkSlaveOne = sparkSlaveOne;
            this.sparkSlaveTwo = sparkSlaveTwo;
        }
    }
}
