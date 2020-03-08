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
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.commands.hood.HoodControl;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Provider;

public class Hood extends SubsystemBase {
    private static final double ZERO_TICKS = 1762;
    private static final double TICKS_PER_DEGREE = 4096.0 / 360;
    private static final int SLOT_IDX = 0;
    public boolean isTrench = false;
    private final WPI_TalonSRX hoodTalon;
    private final LimelightServo servo;
    private final Provider<HoodControl> command;

    public Hood(int motorId, LimelightServo servo, Provider<HoodControl> command) {
        this.servo = servo;
        this.command = command;
        hoodTalon = new WPI_TalonSRX(motorId);
        setupHoodTalon();
    }

    private final void setupHoodTalon() {
        hoodTalon.configFactoryDefault();
        hoodTalon.selectProfileSlot(SLOT_IDX, 0);
        hoodTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
        hoodTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
        hoodTalon.setSensorPhase(false);
        hoodTalon.setNeutralMode(NeutralMode.Brake);
        hoodTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

        hoodTalon.configNominalOutputForward(0);
        hoodTalon.configNominalOutputReverse(0);
        hoodTalon.configPeakOutputForward(1);
        hoodTalon.configPeakOutputReverse(-1);

        hoodTalon.config_kP(SLOT_IDX, (1.5 * 1023 / 400));
        hoodTalon.config_kI(SLOT_IDX, 0);
        hoodTalon.config_kD(SLOT_IDX, 0);
        hoodTalon.config_kF(SLOT_IDX, 0);

        hoodTalon.configMotionCruiseVelocity(800);
        hoodTalon.configMotionAcceleration(800);
    }

    public final WPI_TalonSRX getHoodTalon() {
        return hoodTalon;
    }

    public static double getTicksPerDegree() {
        return TICKS_PER_DEGREE;
    }

    public static double getZeroTicks() {
        return ZERO_TICKS;
    }

    public double getPositionTicks() {
        return hoodTalon.getSensorCollection().getPulseWidthPosition();
    }

    public void setPower(double pwr) {
        pwr = limitPower(pwr);
        hoodTalon.set(ControlMode.PercentOutput, pwr);
    }

    private double limitPower(double pwr) {
        double pos = (getPositionTicks() - HoodPosition.BACK_DEFAULT.ticks)
            / (HoodPosition.FORWARD.ticks - HoodPosition.BACK_DEFAULT.ticks);
        double curvedPwr = Math.pow(Math.E, -(4.0 / 8) * (pos * pos)) * pwr;
        if (limitSafety(curvedPwr)) {
            return curvedPwr;
        } else {
            return 0.0;
        }
    }

    private boolean limitSafety(double pwr) {
        return !(getPositionTicks() <= HoodPosition.BACK_DEFAULT.ticks && pwr < 0)
            && !(getPositionTicks() >= HoodPosition.FORWARD.ticks && pwr > 0);
    }

    public double getAbsoluteAngle() {
        return (getPositionTicks() - ZERO_TICKS) / TICKS_PER_DEGREE;
    }

    public void setAbsoluteAngle(double angle) {
        Robot.getShuffleboard().getTab("Vision Conf").setEntry("SetHoodAngle", angle);
        double ticks = ZERO_TICKS + (angle * TICKS_PER_DEGREE);
        double back = isTrench ? HoodPosition.BACK_TRENCH.ticks : HoodPosition.BACK_DEFAULT.ticks;
        Robot.getShuffleboard().getTab("Vision Conf").setEntry("ticksAngSet", ticks);
        ticks = MathUtil.limit(ticks, back, HoodPosition.FORWARD.ticks);
        Robot.getShuffleboard().getTab("Vision Conf").setEntry("ticksAng", ticks);
        hoodTalon.set(ControlMode.MotionMagic, ticks);
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        servo.setAngle(95 - getAbsoluteAngle());
        super.periodic();
    }
}
