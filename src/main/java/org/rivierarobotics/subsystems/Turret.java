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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.rivierarobotics.commands.turret.TurretControl;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.ShooterUtil;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Provider;

public class Turret extends BasePIDSubsystem {
    private static final double zeroTicks = 3793;
    private static final double maxAngle = 25;
    private static final double minAngle = -243.7;
    private final WPI_TalonSRX turretTalon;
    private final Provider<TurretControl> command;
    private final NavXGyro gyro;
    private final VisionUtil vision;
    private double absolutePosition;

    public Turret(int id, Provider<TurretControl> command, NavXGyro gyro, VisionUtil vision) {
        super(new PIDConfig(0.0017, 0, 0, 0.0, 15, 0.2));
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
            return turretTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getAbsoluteAngle() {
        return ((getPositionTicks() - zeroTicks) * (1 / getAnglesOrInchesToTicks()) + MathUtil.wrapToCircle(gyro.getYaw()));
    }

    public double getAngle() {
        return (getPositionTicks() - zeroTicks) * (1 / getAnglesOrInchesToTicks());
    }

    public double getTxTurret(double distance, double extraDistance) {
        double tx = Math.toRadians(vision.getLLValue("tx"));
        double txTurret = Math.atan2(distance * Math.sin(tx), distance * Math.cos(tx) + extraDistance + ShooterUtil.getLLtoTurretX());
        return txTurret;
    }

    public double getAbsolutePosition() {
        return absolutePosition;
    }

    public void setAbsoluteAngle(double angle) {
        double position = getPositionTicks() + ((angle - getAbsoluteAngle()) * getAnglesOrInchesToTicks());
        absolutePosition = position;
        if (position < zeroTicks + getMaxAngleInTicks() && position > zeroTicks + getMinAngleInTicks()) {
            setPositionTicks(position);
        } else if (position - 4096 < zeroTicks + getMaxAngleInTicks() && position > zeroTicks + getMinAngleInTicks()) {
            setPositionTicks(position - 4096);
        }
    }

    public double getMaxAngleInTicks() {
        return maxAngle * getAnglesOrInchesToTicks();
    }

    public double getMinAngleInTicks() {
        return minAngle * getAnglesOrInchesToTicks();
    }

    @Override
    public void setPower(double pwr) {
        if (pwr <= 0 && getPositionTicks() - zeroTicks < getMinAngleInTicks()) {
            pwr = 0;
        } else if (pwr > 0 && getPositionTicks() - zeroTicks > getMaxAngleInTicks()) {
            pwr = 0;
        }
        turretTalon.set(pwr);
    }

    @Override
    public void setManualPower(double pwr) {
        if (pwr <= 0 && getPositionTicks() - zeroTicks < getMinAngleInTicks()) {
            pwr = 0;
        } else if (pwr > 0 && getPositionTicks() - zeroTicks > getMaxAngleInTicks()) {
            pwr = 0;
        }
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
