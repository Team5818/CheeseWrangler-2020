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
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Provider;

public class Turret extends BasePIDSubsystem {
    private static final double zeroTicks = 1383;
    private final WPI_TalonSRX turretTalon;
    private final Provider<TurretControl> command;
    private final NavXGyro gyro;
    private final VisionUtil vision;
    public AimMode mode = AimMode.ENCODER;
    private double angle;
    private static final double maxAngle = 150;

    public Turret(int id, Provider<TurretControl> command, NavXGyro gyro, VisionUtil vision) {
        //TODO: more tuning :):):):):):)
        super(new PIDConfig(0.001, 0.0, 0.00000, 0.0, 15, 1.0));
        this.command = command;
        this.gyro = gyro;
        this.vision = vision;
        turretTalon = new WPI_TalonSRX(id);
        turretTalon.configFactoryDefault();
        turretTalon.setSensorPhase(false);
        turretTalon.setInverted(true);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    }

    public double getVelocity() {
        double pos = turretTalon.getSensorCollection().getPulseWidthVelocity();
        if (mode == AimMode.VISION) {
            pos = -vision.getLLValue("tx") * getAnglesOrInchesToTicks();
        }
        SmartDashboard.putNumber("Position", pos);
        SmartDashboard.putBoolean("atSetpoint", getPidController().atSetpoint());
        SmartDashboard.putNumber("setpoint", getPidController().getSetpoint());
        return pos;
    }

    @Override
    public double getPositionTicks() {
        if (mode == AimMode.MOVING) {
            return turretTalon.getSensorCollection().getPulseWidthVelocity();
        } else {

            return turretTalon.getSensorCollection().getPulseWidthPosition();
        }


    }

    public double getAbsoluteAngle() {
        return ((getPositionTicks() - zeroTicks) * (1 / getAnglesOrInchesToTicks()) + MathUtil.wrapToCircle(gyro.getYaw()));
    }

    public double getAngle() {
        return (getPositionTicks() - zeroTicks) * (1 / getAnglesOrInchesToTicks());
    }

    public double getTxTurret(double distance, double extraDistance) {
        double tx = Math.toRadians(vision.getLLValue("tx") + getAbsoluteAngle());
        double txTurret = Math.atan2(distance * Math.sin(tx) - 0.18, distance * Math.cos(tx) + extraDistance);
        SmartDashboard.putNumber("Modified tx", Math.toDegrees(tx));
        SmartDashboard.putNumber("txTurret", Math.toDegrees(txTurret));
        return txTurret;
    }

    public void setAbsolutePosition(double angle) {
        this.angle = angle;
        SmartDashboard.putNumber("Turret SetAngle", angle);
        double position = getPositionTicks() + ((angle - getAbsoluteAngle()) * getAnglesOrInchesToTicks());
        SmartDashboard.putNumber("turretset", position);
        if (position < zeroTicks + maxAngle * getAnglesOrInchesToTicks() && position > zeroTicks - getMaxAngleInTicks()) {
            setPositionTicks(position);
        } else {
            position -= 4096;
            if (position < zeroTicks + maxAngle * getAnglesOrInchesToTicks() && position > zeroTicks - getMaxAngleInTicks()) {
                setPositionTicks(position);
            } else {
                return;
            }
        }
    }

    public double getMaxAngleInTicks() {
        return maxAngle * getAnglesOrInchesToTicks();
    }

    @Override
    public void setPower(double pwr) {
        turretTalon.set(pwr);
    }

    @Override
    public void setManualPower(double pwr) {
        if (pwr <= 0 && getPositionTicks() - zeroTicks < -getMaxAngleInTicks()) {
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

    public void changeAimMode(AimMode mode) {
        this.mode = mode;
    }


    public enum AimMode {
        VISION, ENCODER, MOVING, STILL;
    }
}
