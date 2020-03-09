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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import org.rivierarobotics.util.NeutralIdleMode;

public class DriveTrainSide implements RRSubsystem {
    // same units as distance per pulse
    private static final double MAX_VELOCITY = 5.7912;
    private static final double FEED_FORWARD = 1.0 / MAX_VELOCITY;
    private final PIDController pidController;
    private WPI_TalonFX leftMotor;
    private WPI_TalonFX rightMotor;
    private Encoder shaftEncoder;
    private final boolean invert;
    private boolean pidEnabled;
    private final double pidRange;

    public DriveTrainSide(DTMotorIds motors, boolean invert) {
        this.leftMotor = new WPI_TalonFX(motors.left);
        this.rightMotor = new WPI_TalonFX(motors.right);
        this.invert = invert;
        this.pidController = new PIDController(0, 0, 0, 0.02);
        this.pidController.setTolerance(5);
        this.pidRange = 1.0;

        setupMotors(leftMotor, rightMotor);
        NeutralIdleMode.BRAKE.applyTo(leftMotor, rightMotor);

        this.shaftEncoder = new Encoder(motors.encoderA, motors.encoderB);
        // meters / ticks
        shaftEncoder.setDistancePerPulse(-1.8288 / 7916);
    }

    private void setupMotors(WPI_TalonFX... motors) {
        for (WPI_TalonFX motor : motors) {
            motor.configFactoryDefault();
            motor.setInverted(invert);
            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        }
    }

    @Override
    public double getPositionTicks() {
        return shaftEncoder.getRaw();
    }

    @Override
    public void setPower(double pwr) {
        leftMotor.set(pwr);
        rightMotor.set(pwr);
    }

    public double getPosition() {
        return shaftEncoder.getDistance();
    }

    public double getVelocity() {
        return shaftEncoder.getRate();
    }

    public void setVelocity(double vel) {
        pidEnabled = true;
        pidController.setSetpoint(vel);
    }

    public void setManualPower(double pwr) {
        pidEnabled = false;
        setPower(pwr);
    }

    public void tickPid() {
        if (pidEnabled) {
            double pidPower = Math.min(pidRange, Math.max(-pidRange, pidController.calculate(getPosition())));
            pidPower += pidController.getSetpoint() * FEED_FORWARD;
            setPower(pidPower);
        }
    }

    public void setNeutralIdle(NeutralIdleMode mode) {
        mode.applyTo(leftMotor, rightMotor);
    }

    public void resetEncoder() {
        shaftEncoder.reset();
    }
}
