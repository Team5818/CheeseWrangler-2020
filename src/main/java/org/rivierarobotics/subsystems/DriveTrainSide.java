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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.MotorUtil;

public class DriveTrainSide implements RRSubsystem {
    private static final double TICKS_PER_METER = 7916 / 1.8288;
    private static final double MOTOR_TO_WHEEL_RATIO = 17.0 / 48;
    private final WPI_TalonFX masterLeft;
    private final WPI_TalonFX slaveRight;
    private final Encoder shaftEncoder;
    private final MechLogger logger;
    private final Boolean inverted;

    public DriveTrainSide(DTMotorIds motors, boolean invert) {
        this.masterLeft = new WPI_TalonFX(motors.master);
        this.slaveRight = new WPI_TalonFX(motors.slave);
        this.inverted = invert;
        this.logger = Logging.getLogger(getClass(), invert ? "left" : "right");

        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor,
            new PIDConfig(0.05, 0, 0, 0), 0, masterLeft, slaveRight);
        masterLeft.setInverted(invert);
        slaveRight.setInverted(invert);
        masterLeft.setNeutralMode(NeutralMode.Brake);
        slaveRight.setNeutralMode(NeutralMode.Brake);
        slaveRight.follow(masterLeft);

        this.shaftEncoder = new Encoder(motors.encoderA, motors.encoderB);
        shaftEncoder.setReverseDirection(true);
        shaftEncoder.setDistancePerPulse(1 / TICKS_PER_METER);
    }

    @Override
    public double getPositionTicks() {
        return getPosition() * TICKS_PER_METER;
    }

    @Override
    public void setPower(double pwr) {
        masterLeft.set(TalonFXControlMode.PercentOutput, pwr);
    }

    public double getLVoltage() {
        return masterLeft.getMotorOutputVoltage();
    }

    public double getRVoltage() {
        return slaveRight.getMotorOutputVoltage();
    }

    public double getPosition() {
        return shaftEncoder.getDistance();
    }

    public double getVelocity() {
        return shaftEncoder.getRate();
    }

    public void setVelocity(double vel) {
        // Converts m/s to ticks/100ms and sets velocity
        double set = (vel * TICKS_PER_METER) / (MOTOR_TO_WHEEL_RATIO * 10);
        logger.setpointChange(set);
        masterLeft.set(ControlMode.Velocity, set);
    }

    public void setVoltage(double volts) {
//        volts = !inverted ? -volts : volts;
        masterLeft.setVoltage(volts);
        slaveRight.setVoltage(volts);
    }

    public void resetEncoder() {
        shaftEncoder.reset();
    }
}
