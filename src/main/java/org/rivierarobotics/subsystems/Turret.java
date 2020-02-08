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
import org.rivierarobotics.commands.TurretControl;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Provider;

public class Turret extends BasePIDSubsystem {
    private static final double zeroTicks = 1383;
    private final WPI_TalonSRX turretTalon;
    private final Provider<TurretControl> command;
    private final NavXGyro gyro;
    private final VisionUtil vision;

    public Turret(int id, Provider<TurretControl> command, NavXGyro gyro, VisionUtil vision) {
//        super(0.0025, 0.000, 0.0, 1.0);
        super(0.05, 0.0, 0.0, 0.2);
        this.command = command;
        this.gyro = gyro;
        this.vision = vision;

        turretTalon = new WPI_TalonSRX(id);
        turretTalon.configFactoryDefault();
        turretTalon.setSensorPhase(false);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    }

    @Override
    public double getPositionTicks() {
//        double pos = turretTalon.getSensorCollection().getPulseWidthPosition();
        double pos = -vision.getLLValue("tx");
        SmartDashboard.putNumber("Position", pos);
        SmartDashboard.putBoolean("atSetpoint", getPidController().atSetpoint());
        SmartDashboard.putNumber("setpoint", getPidController().getSetpoint());
        return pos;
    }

    public double getAbsoluteAngle() {
        return ((getPositionTicks() - zeroTicks) * 360 / 4096.0 + gyro.getYaw());
    }

    public double getAbsoluteTicks() {
        return getPositionTicks() - zeroTicks + (gyro.getYaw() * getAnglesOrInchesToTicks());
    }

    public void setAbsolutePosition(double angle) {
        setPositionTicks(getPositionTicks() + ((angle - getAbsoluteAngle()) * getAnglesOrInchesToTicks()));
    }

    @Override
    public void setPower(double pwr) {
        turretTalon.set(pwr);
    }

    @Override
    public void setManualPower(double pwr) {
        if (((getPositionTicks()  < -150 * getAnglesOrInchesToTicks() && pwr < 0)
                || (getPositionTicks() - zeroTicks > 150 * getAnglesOrInchesToTicks() && pwr > 0))) {
            pwr = 0;
        }
        SmartDashboard.putNumber("turretpwr", pwr);
        super.setManualPower(pwr);
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
