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

import static org.rivierarobotics.util.Dimensions.WHEEL_CIRCUMFERENCE;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import org.rivierarobotics.util.NeutralIdleMode;

public class DriveTrainSide extends BasePIDSubsystem {
    // TODO re-find ticks-per-inch for the comp bot
    private static final double TICKS_PER_INCH = 12720.0 / 72;
    private final boolean invert;
    private WPI_TalonFX tl;
    private WPI_TalonFX tr;
    private Encoder shaftEncoder;

    public DriveTrainSide(DTMotorIds motors, boolean invert) {
        super(new PIDConfig(0.0, 0.0, 0.0, 0.05, 0.0, 1.0)); //kF is not accurate
        this.tl = new WPI_TalonFX(motors.topLeft);
        this.tr = new WPI_TalonFX(motors.topRight);
        this.invert = invert;

        setupMotors(tl, tr);
        NeutralIdleMode.COAST.applyTo(tl, tr);

        this.shaftEncoder = new Encoder(motors.encoderA, motors.encoderB);
        shaftEncoder.setDistancePerPulse(WHEEL_CIRCUMFERENCE / 2048);
    }

    private void setupMotors(WPI_TalonFX... motors) {
        for (WPI_TalonFX motor : motors) {
            motor.configFactoryDefault();
            motor.setInverted(invert);
            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        }
    }

    public void setPower(double pwr) {
        tl.set(pwr);
        tr.set(pwr);
    }

    public double getPositionTicks() {
        return getVelocity();
    }

    public double getPosition() {
        return shaftEncoder.getDistance();
    }

    public double getVelocity() {
        return shaftEncoder.getRate();
    }

    public void setVelocity(double vel) {
        // TODO implement proper velocity control
    }

    public void setNeutralIdle(NeutralIdleMode mode) {
        mode.applyTo(tl, tr);
    }

    public void resetEncoder() {
        shaftEncoder.reset();
    }
}
