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
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.MotorUtil;
import org.rivierarobotics.util.NeutralIdleMode;

public class DriveTrainSide implements RRSubsystem {
    private static final double TICKS_PER_METER = 7916 / 1.8288;
    private static final double MOTOR_TO_WHEEL_RATIO = 17.0 / 48;
    private WPI_TalonFX masterLeft;
    private WPI_TalonFX slaveRight;
    private Encoder shaftEncoder;

    public DriveTrainSide(DTMotorIds motors, boolean invert) {
        this.masterLeft = new WPI_TalonFX(motors.left);
        this.slaveRight = new WPI_TalonFX(motors.right);

        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor,
            new PIDConfig(1023 * 0.2, 0, 0, 0), 0, masterLeft, slaveRight);
        MotorUtil.setInverted(invert, masterLeft, slaveRight);
        NeutralIdleMode.BRAKE.applyTo(masterLeft, slaveRight);
        slaveRight.follow(masterLeft);

        this.shaftEncoder = new Encoder(motors.encoderA, motors.encoderB);
        shaftEncoder.setDistancePerPulse(1 / TICKS_PER_METER);
    }

    @Override
    public double getPositionTicks() {
        return masterLeft.getSelectedSensorVelocity();
    }

    @Override
    public void setPower(double pwr) {
        masterLeft.set(TalonFXControlMode.PercentOutput, pwr);
    }

    public double getPosition() {
        return shaftEncoder.getDistance();
    }

    // Returns in meters per second
    public double getVelocity() {
        return MathUtil.ticksPer100msToMetersPerSec(
            masterLeft.getSelectedSensorVelocity() * MOTOR_TO_WHEEL_RATIO, TICKS_PER_METER);
    }

    public void setVelocity(double vel) {
        masterLeft.set(TalonFXControlMode.Velocity, MathUtil.metersPerSecToTicksPer100ms(
            vel / MOTOR_TO_WHEEL_RATIO, TICKS_PER_METER));
    }

    public void setNeutralIdle(NeutralIdleMode mode) {
        mode.applyTo(masterLeft, slaveRight);
    }

    public void resetEncoder() {
        shaftEncoder.reset();
    }
}
