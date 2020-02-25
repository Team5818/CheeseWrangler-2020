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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import org.rivierarobotics.util.NeutralIdleMode;

public class DriveTrainSide {
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

    public DriveTrainSide(MotorIds motors, boolean invert) {
        this.tl = new WPI_TalonFX(motors.topLeft);
        this.tr = new WPI_TalonFX(motors.topRight);
        this.bl = new WPI_TalonFX(motors.bottomLeft);
        this.br = new WPI_TalonFX(motors.bottomRight);
        this.currentGear = DriveTrain.Gear.HYBRID;
        this.invert = invert;

        setupMotors(tl, tr, bl, br);
        NeutralIdleMode.COAST.applyTo(tl, tr, bl, br);

        this.shaftEncoder = new Encoder(motors.encoderA, motors.encoderB);
    }

    private void setupMotors(WPI_TalonFX... motors) {
        for (WPI_TalonFX motor : motors) {
            motor.configFactoryDefault();
            motor.setInverted(invert);
        }
    }

    public void setVelocity(double vel) {
        // TODO implement proper velocity control
        throw new UnsupportedOperationException();
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
        return shaftEncoder.get();
    }

    //TODO determine what units this is in
    public double getPosition() {
        return shaftEncoder.getDistance();
    }

    //TODO determine what units this is in
    public double getVelocity() {
        return shaftEncoder.getRate();
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

    public static class MotorIds {
        public final int topLeft;
        public final int topRight;
        public final int bottomLeft;
        public final int bottomRight;
        public final int encoderA;
        public final int encoderB;

        public MotorIds(int topLeft, int topRight, int bottomLeft, int bottomRight, int encoderA, int encoderB) {
            this.topLeft = topLeft;
            this.topRight = topRight;
            this.bottomLeft = bottomLeft;
            this.bottomRight = bottomRight;
            this.encoderA = encoderA;
            this.encoderB = encoderB;
        }
    }
}
