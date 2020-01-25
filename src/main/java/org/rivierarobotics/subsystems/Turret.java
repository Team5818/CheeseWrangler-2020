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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.rivierarobotics.commands.TurretControl;
import org.rivierarobotics.util.RobotMap;

public class Turret extends BasePID implements Subsystem {
    private final WPI_TalonSRX turretTalon;
    private static final double zeroticks = 1186;

    public Turret() {
        //TODO tune Turret PID
        super(0.0015, 0.00002, 0.0, 0.5, 0.0, "Turret");
        turretTalon = new WPI_TalonSRX(RobotMap.Controllers.TURRET_TALON);
        turretTalon.configFactoryDefault();
        turretTalon.setSensorPhase(true);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        setDefaultCommand(new TurretControl(this));
//        getPidController().enableContinuousInput(0, 4096);
    }

    @Override
    public double getPositionTicks() {
        //TODO ensure that this reports correctly: potential fix for digital encoder jumping issues, eliminates some high bits
        double pos = turretTalon.getSensorCollection().getPulseWidthPosition();
        SmartDashboard.putNumber("Position", pos);
        SmartDashboard.putNumber("RisetoFall", turretTalon.getSensorCollection().getPulseWidthRiseToFallUs());
        SmartDashboard.putNumber("Circle position", (pos % 4096));
        SmartDashboard.putBoolean("atSetpoint", getPidController().atSetpoint());
        SmartDashboard.putNumber("setpoint", getPidController().getSetpoint());
        return pos;
    }

    public void setAbsoluteAngle(double angle) {
        setTicksPosition((angle * getAnglesOrInchesToTicks()) + zeroticks);
    }

    @Override
    public void setPower(double pwr) {
        SmartDashboard.putNumber("power", pwr);
        turretTalon.set(pwr);
    }
}
