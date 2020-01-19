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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.util.RobotMap;

public class Turret extends BasePIDSubsystem {
    private final WPI_TalonSRX turretTalon;
    private final AnalogInput sensor;

    public Turret() {
        super(0.0004, 0, 0.0, 0.4, 1023.0 / 360);
        turretTalon = new WPI_TalonSRX(RobotMap.TURRET_TALON);
        sensor = new AnalogInput(0);
        turretTalon.configFactoryDefault();
        turretTalon.setSensorPhase(false);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        getPidController().setTolerance(0);
        getPidController().enableContinuousInput(0, 1023);
    }

    @Override
    public void setPosition(double position) {
        SmartDashboard.putNumber("turret setpoint", position);
        SmartDashboard.putBoolean("atSetpoint", getPidController().atSetpoint());
        super.setPosition(position);
    }

    @Override
    public double getPositionTicks() {
        double pos = sensor.getAverageValue();
        SmartDashboard.putNumber("turret pos", pos);
        return pos;
    }

    @Override
    public void setPower(double pwr) {
        SmartDashboard.putNumber("turret pwr", pwr);
        turretTalon.set(pwr);
        getPositionTicks();
    }
}
