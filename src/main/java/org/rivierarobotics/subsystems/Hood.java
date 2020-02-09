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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.commands.HoodControl;

import javax.inject.Provider;

public class Hood extends BasePIDSubsystem {
    private final WPI_TalonSRX hoodTalon;
    private final Provider<HoodControl> command;
    private final DigitalInput limit;
    private static final double zeroTicks = -2334;

    public Hood(int motorId, int limitId, Provider<HoodControl> command) {
        super(0.0016 , 0.0, 0.0, 1.0);
        this.command = command;
        hoodTalon = new WPI_TalonSRX(motorId);
        limit = new DigitalInput(limitId);
        hoodTalon.configFactoryDefault();
        hoodTalon.setSensorPhase(true);
        hoodTalon.setNeutralMode(NeutralMode.Brake);
    }

    public WPI_TalonSRX getHoodTalon() {
        return hoodTalon;
    }

    @Override
    public double getPositionTicks() {
        return hoodTalon.getSensorCollection().getQuadraturePosition();
    }

    @Override
    public void setPower(double pwr) {
        hoodTalon.set(pwr);
    }

    public void setAbsolutePosition(double angle) {
        SmartDashboard.putNumber("SetHoodAngle", angle);
        if ( angle >= -20 && angle <= 40) {
            SmartDashboard.putNumber("Hood SetTicks", zeroTicks + angle * getAnglesOrInchesToTicks() * -5);
            setPositionTicks(zeroTicks + (angle * getAnglesOrInchesToTicks()) * 5);
        }
    }

    public double getAbsolutePosition() {
        return (zeroTicks - getPositionTicks()) / 5 * 360 / 4096;
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }

    public boolean isAtEnd() {
        // switch is false when triggered
        return !limit.get();
    }
}
