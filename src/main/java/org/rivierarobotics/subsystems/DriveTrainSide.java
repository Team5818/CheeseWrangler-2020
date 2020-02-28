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
    //TODO change values of threshold to realistic values
    private static final double LOW_BOUND_LOW = 5000 / 2.0;
    private static final double HIGH_BOUND_LOW = 6000 / 2.0;
    private static final double LOW_BOUND_HIGH = LOW_BOUND_LOW / 2.5;
    private static final double HIGH_BOUND_HIGH = (HIGH_BOUND_LOW / 2.5);
    // TODO re-find ticks-per-inch for the comp bot
    private static final double TICKS_PER_INCH = 12720.0 / 72;
    private final boolean invert;
    private WPI_TalonFX tl;
    private WPI_TalonFX tr;
    private WPI_TalonFX bl;
    private WPI_TalonFX br;
    private Encoder shaftEncoder;
    private DriveTrain.Gear currentGear;

    public DriveTrainSide(DTMotorIds motors, boolean invert) {
        super(new PIDConfig(0.0, 0.0, 0.0, 0.05, 0.0, 1.0)); //kF is not accurate
        this.tl = new WPI_TalonFX(motors.topLeft);
        this.tr = new WPI_TalonFX(motors.topRight);
        this.bl = new WPI_TalonFX(motors.bottomLeft);
        this.br = new WPI_TalonFX(motors.bottomRight);
        this.currentGear = DriveTrain.Gear.HYBRID;
        this.invert = invert;

        setupMotors(tl, tr, bl, br);
        NeutralIdleMode.COAST.applyTo(tl, tr, bl, br);

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
        setPower(pwr, getRPMHigh(), getRPMLow());
    }

    public void setPower(double pwr, double highRPM, double lowRPM) {
        switch (currentGear) {
            case HYBRID:
                hybridSetPower(pwr, highRPM, lowRPM);
                break;
            case LOW:
                bl.set(pwr);
                br.set(pwr);
                break;
            case HIGH:
                tl.set(pwr);
                tr.set(pwr);
                break;
            default:
                throw new IllegalStateException("Unknown gear: " + currentGear);
        }
    }

    private void hybridSetPower(double pwr, double highRPM, double lowRPM) {
        double highPower = 0;
        if ((highRPM > LOW_BOUND_HIGH && highRPM < HIGH_BOUND_HIGH)) {
            highPower = ((HIGH_BOUND_HIGH - highRPM) / (HIGH_BOUND_HIGH - LOW_BOUND_HIGH)) * pwr;
        } else if (highRPM > HIGH_BOUND_HIGH) {
            highPower = pwr;
        }
        tl.set(highPower);
        tr.set(highPower);

        double lowPower = 0;
        if (lowRPM < LOW_BOUND_LOW) {
            lowPower = pwr;
        }
        bl.set(lowPower);
        br.set(lowPower);
    }

    public double getPositionTicks() {
        return getVelocity();
    }

    public double getPosition() {
        return shaftEncoder.getRaw();
    }

    public double getVelocity() {
        return shaftEncoder.getRate();
    }

    public void setVelocity(double vel) {
        // TODO implement proper velocity control
    }

    public double getMotorRPM(WPI_TalonFX motor) {
        return motor.getSensorCollection().getIntegratedSensorVelocity() * (600.0 / 2048);
    }

    public double getRPMHigh() {
        return (Math.abs(getMotorRPM(tl)) + Math.abs(getMotorRPM(tr))) / 2;
    }

    public double getRPMLow() {
        return (Math.abs(getMotorRPM(bl)) + Math.abs(getMotorRPM(br))) / 2;
    }

    public void setGear(DriveTrain.Gear gear) {
        this.currentGear = gear;
    }

    public void setNeutralIdle(NeutralIdleMode mode) {
        mode.applyTo(tl, tr, bl, br);
    }

    public void resetEncoder() {
        shaftEncoder.reset();
    }
}
