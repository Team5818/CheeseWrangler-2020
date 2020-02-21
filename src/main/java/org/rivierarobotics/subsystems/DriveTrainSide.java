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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.followers.EncoderFollower;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.NeutralIdleMode;

public class DriveTrainSide {
    //TODO change values of threshold to realistic values
    private final double lowBoundLow = 5000 / 2.0, highBoundLow = 6000 / 2.0,
            lowBoundHigh = lowBoundLow / 2.5, highBoundHigh = (highBoundLow / 2.5);
    private final boolean invert;
    private WPI_TalonFX tl, tr, bl, br;
    private Encoder shaftEncoder;
    private DriveTrain.Gear currentGear;
    private EncoderFollower follower;

    public DriveTrainSide(MotorIds motors, boolean invert) {
        this.tl = new WPI_TalonFX(motors.tl);
        this.tr = new WPI_TalonFX(motors.tr);
        this.bl = new WPI_TalonFX(motors.bl);
        this.br = new WPI_TalonFX(motors.br);
        this.currentGear = DriveTrain.Gear.HYBRID;
        this.invert = invert;

        setupMotors(tl, tr, bl, br);
        NeutralIdleMode.COAST.applyTo(tl, tr, bl, br);

        this.shaftEncoder = new Encoder(motors.encA, motors.encB);

        this.follower = new EncoderFollower();
        follower.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);
    }

    private void setupMotors(WPI_TalonFX... motors) {
        for (WPI_TalonFX motor : motors) {
            motor.configFactoryDefault();
            motor.setInverted(invert);
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
        }
    }

    private void hybridSetPower(double pwr, double highRPM, double lowRPM) {
        double highPower = 0;
        if ((highRPM > lowBoundHigh && highRPM < highBoundHigh)) {
            highPower = ((highBoundHigh - highRPM) / (highBoundHigh - lowBoundHigh)) * pwr;
        } else if (highRPM > highBoundHigh) {
            highPower = pwr;
        }
        tl.set(highPower);
        tr.set(highPower);

        double lowPower = 0;
        if (lowRPM < lowBoundLow) {
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

    public EncoderFollower getEncoderFollower() {
        return follower;
    }

    public static class MotorIds {
        public final int tl, tr, bl, br, encA, encB;

        public MotorIds(int tl, int tr, int bl, int br, int encA, int encB) {
            this.tl = tl;
            this.tr = tr;
            this.bl = bl;
            this.br = br;
            this.encA = encA;
            this.encB = encB;
        }
    }
}
