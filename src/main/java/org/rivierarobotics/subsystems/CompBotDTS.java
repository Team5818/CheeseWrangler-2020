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
import jaci.pathfinder.followers.EncoderFollower;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.NeutralIdleMode;

public class CompBotDTS {
    private WPI_TalonFX tl, tr, bl, br;
    private Encoder shaftEncoder;
    private DriveTrainGear currentGear;
    private EncoderFollower follower;
    //TODO change values of threshold to realistic values
    private double lowHybridThreshold = 100, highHybridThreshold = 200;

    public CompBotDTS(CompBotDTSMotorIds motors, boolean invert) {
        this.tl = new WPI_TalonFX(motors.tl);
        this.tr = new WPI_TalonFX(motors.tr);
        this.bl = new WPI_TalonFX(motors.bl);
        this.br = new WPI_TalonFX(motors.br);
        this.currentGear = DriveTrainGear.HYBRID;

        setupMotors(tl, tr, bl, br);
        NeutralIdleMode.BRAKE.applyTo(tl, tr, bl, br);
        //TODO determine inversion states of different motors
        tl.setInverted(invert);
        tr.setInverted(invert);
        bl.setInverted(invert);
        br.setInverted(invert);

        this.shaftEncoder = new Encoder(motors.encA, motors.encB);

        this.follower = new EncoderFollower();
        follower.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0);
    }

    private void setupMotors(WPI_TalonFX... motors) {
        for (WPI_TalonFX motor : motors) {
            motor.configFactoryDefault();
            //TODO add more motor configuration
        }
    }

    //TODO check the math on this
    public void setPower(double pwr) {
        double vel = getVelocity();
        switch (currentGear) {
            case HYBRID:
                if (vel < lowHybridThreshold) {
                    bl.set(pwr);
                    br.set(pwr);
                } else if (vel < highHybridThreshold) {
                    double pwrHigh = MathUtil.limit(((highHybridThreshold - vel) / 100) * pwr, 1.0);
                    double pwrLow = MathUtil.limit(((vel - lowHybridThreshold) / 100) * pwr, 1.0);

                    bl.set(pwrLow);
                    br.set(pwrLow);
                    tl.set(pwrHigh);
                    tl.set(pwrHigh);
                } else {
                    tl.set(pwr);
                    tr.set(pwr);
                }
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

    public void setGear(DriveTrainGear gear) {
        this.currentGear = gear;
        switch (gear) {
            case HYBRID:
                NeutralIdleMode.BRAKE.applyTo(bl, br, tl, tr);
            case LOW:
                NeutralIdleMode.BRAKE.applyTo(bl, br);
                NeutralIdleMode.COAST.applyTo(tl, tr);
                break;
            case HIGH:
                NeutralIdleMode.BRAKE.applyTo(tl, tr);
                NeutralIdleMode.COAST.applyTo(bl, br);
                break;
        }
    }

    public EncoderFollower getEncoderFollower() {
        return follower;
    }
}