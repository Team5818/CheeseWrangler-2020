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
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.ShooterConstants;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Provider;

public class Turret extends BasePIDSubsystem implements RRSubsystem {
    private static final double zeroTicks = 3793;
    private static final double maxAngle = 25;
    private static final double minAngle = -243.7;
    private final WPI_TalonSRX turretTalon;
    private final RobotShuffleboard shuffleboard;
    private final Provider<TurretControl> command;
    private final NavXGyro gyro;
    private final VisionUtil vision;
    private static boolean isAutoAimEnabled;

    public Turret(int id, Provider<TurretControl> command, NavXGyro gyro, VisionUtil vision, RobotShuffleboard shuffleboard) {
        super(new PIDConfig(0.0017, 0, 0, 0.0, 15, 0.5));
        this.command = command;
        this.gyro = gyro;
        this.vision = vision;
        this.shuffleboard = shuffleboard;
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
        return MathUtil.wrapToCircle((getPositionTicks() - zeroTicks) * (1 / getTicksPerAngleOrInch()) + (gyro.getYaw()));
    }

    public double getAngle() {
        return (getPositionTicks() - zeroTicks) * (1 / getTicksPerAngleOrInch());
    }

    public double getTxTurret(double distance, double extraDistance) {
        double tx = Math.toRadians(vision.getLLValue("tx"));
        double txTurret = Math.atan2(distance * Math.sin(tx) + ShooterConstants.getLLtoTurretZ(), distance * Math.cos(tx) + extraDistance);
        shuffleboard.getTab("TurretHood").setEntry("txTurret", txTurret);
        return txTurret;
    }

    public void setAngle(double angle) {
        double position = getPositionTicks() + ((angle - getAbsoluteAngle()) * getTicksPerAngleOrInch());
        if (position < zeroTicks + getMaxAngleInTicks() && position > zeroTicks + getMinAngleInTicks()) {
            logger.setpointChange(position);
            setPositionTicks(position);
        } else if (position - 4096 < zeroTicks + getMaxAngleInTicks() && position > zeroTicks + getMinAngleInTicks()) {
            logger.setpointChange(position - 4096);
            setPositionTicks(position - 4096);
        }
    }

    public void enableAutoAim() {
        isAutoAimEnabled = true;
    }

    public void disableAutoAim() {
        isAutoAimEnabled = false;
    }

    public boolean isAutoAimEnabled() {
        return isAutoAimEnabled;
    }

    public double getMaxAngleInTicks() {
        return maxAngle * getTicksPerAngleOrInch();
    }

    public double getMinAngleInTicks() {
        return minAngle * getTicksPerAngleOrInch();
    }

    private double limitPowerToRange(double pwr) {
        if (pwr <= 0 && getPositionTicks() - zeroTicks < getMinAngleInTicks()) {
            pwr = 0;
        } else if (pwr > 0 && getPositionTicks() - zeroTicks > getMaxAngleInTicks()) {
            pwr = 0;
        }
        return pwr;
    }

    @Override
    public void setPower(double pwr) {
        pwr = limitPowerToRange(pwr);
        logger.powerChange(pwr);
        turretTalon.set(pwr);
    }

    @Override
    public void setManualPower(double pwr) {
        super.setManualPower(limitPowerToRange(pwr));
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
