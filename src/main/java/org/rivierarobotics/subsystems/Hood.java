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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.commands.HoodControl;

import javax.inject.Provider;

public class Hood extends BasePIDSubsystem {
    private static final double zeroTicks = -2334;
    private final WPI_TalonFX hoodFalcon;
    private final Provider<HoodControl> command;
    private final DigitalInput limit;

    public Hood(int motorId, int limitId, Provider<HoodControl> command) {
        super(new PIDConfig(0.001, 0.0001, 0.0, 0.03, 10, 0.6), 4096 / 360.0);
        this.command = command;
        hoodFalcon = new WPI_TalonFX(motorId);
        limit = new DigitalInput(limitId);
        hoodFalcon.configFactoryDefault();
        hoodFalcon.setSensorPhase(true);
        hoodFalcon.setNeutralMode(NeutralMode.Brake);
    }

    public final WPI_TalonFX getHoodFalcon() {
        return hoodFalcon;
    }

    public boolean isAtEnd() {
        // switch is false when triggered
        return !limit.get();
    }

    @Override
    public double getPositionTicks() {
        return hoodFalcon.getSensorCollection().getIntegratedSensorPosition();
    }

    @Override
    public void setPower(double pwr) {
        hoodFalcon.set(pwr);
    }

    @Override
    public void setManualPower(double pwr) {
        if (pwr >= 0 && getPositionTicks() > 0) {
            pwr = 0;
        } else if (pwr < 0 && getPositionTicks() < -4000) {
            pwr = 0;
        }
        SmartDashboard.putNumber("HoodPower", pwr);
        super.setManualPower(pwr);
    }

    public double getAbsolutePosition() {
        return (zeroTicks - getPositionTicks()) / 5 * 360 / 4096;
    }

    //TODO attempt to eliminate field "angle" as it appears to not be needed
    public void setAbsoluteAngle(double angle) {
        SmartDashboard.putNumber("SetHoodAngle", angle);
        if (angle >= -20 && angle <= 42) {
            SmartDashboard.putNumber("Hood SetTicks", zeroTicks + angle * getAnglesOrInchesToTicks() * -5);
            setPositionTicks(zeroTicks + (angle * getAnglesOrInchesToTicks()) * 5);
        } else {
            setPositionTicks(0);
        }
    }
}
