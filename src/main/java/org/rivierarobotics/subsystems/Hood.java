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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import org.rivierarobotics.commands.HoodControl;

import javax.inject.Provider;

public class Hood extends BasePIDSubsystem {
    private static final double LOWER_HARD = 2028;
    private static final double LOWER_SOFT = 2158;
    private static final double HIGHER_HARD = 2410;
    private static final double HIGHER_SOFT = 2250;
    private static final double zeroTicks = 2786;
    private final WPI_TalonSRX hoodTalon;
    private final Provider<HoodControl> command;

    public Hood(int motorId, Provider<HoodControl> command) {
        super(new PIDConfig(0.0015, 0.001, 0.0, 0.0, 10, 0.4), 4096 / 360.0);
        this.command = command;
        hoodTalon = new WPI_TalonSRX(motorId);
        hoodTalon.configFactoryDefault();
        hoodTalon.setSensorPhase(true);
        hoodTalon.setNeutralMode(NeutralMode.Brake);
    }

    public final WPI_TalonSRX getHoodTalon() {
        return hoodTalon;
    }

    @Override
    public double getPositionTicks() {
        return hoodTalon.getSensorCollection().getPulseWidthPosition();
    }

    @Override
    public void setPower(double pwr) {
        pwr = limitPower(pwr);
        hoodTalon.set(pwr);
    }

    @Override
    public void setManualPower(double pwr) {
        pwr = limitPower(pwr);
        super.setManualPower(pwr);
    }

    private double limitPower(double pwr) {
        if (pwr <= 0 && getPositionTicks() < LOWER_SOFT) {
            pwr = MathUtil.clamp(pwr, 0.6 * -(getPositionTicks() - LOWER_HARD) / (LOWER_SOFT - LOWER_HARD), 0);
        } else if (pwr > 0 && getPositionTicks() > HIGHER_SOFT) {
            pwr = MathUtil.clamp(pwr, 0, 0.6 * (HIGHER_HARD - getPositionTicks()) / (HIGHER_HARD - HIGHER_SOFT));
        }
        return Math.signum(pwr) * MathUtil.clamp(Math.abs(pwr), 0, 0.6);
    }

    public double getAbsolutePosition() {
        return (zeroTicks - getPositionTicks()) * 360 / 4096;
    }

    public void setAbsoluteAngle(double angle) {
        SmartDashboard.putNumber("SetHoodAngle", angle);
        if (angle >= 33 && angle <= 66) {
            SmartDashboard.putNumber("Hood SetTicks", zeroTicks - angle * getAnglesOrInchesToTicks());
            setPositionTicks(zeroTicks + (angle * getAnglesOrInchesToTicks()));
        } else {
            setPositionTicks(0);
        }
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
