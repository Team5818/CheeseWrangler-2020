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

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.turret.TurretControl;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.MotorUtil;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;
import org.rivierarobotics.util.ShooterConstants;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Provider;

public class Turret extends SubsystemBase implements RRSubsystem {
    private static final double ZERO_TICKS = 3692;
    private static final double MAX_ANGLE = 25;
    private static final double MIN_ANGLE = -120;
    private static final double TICKS_PER_DEGREE = 4096.0 / 360;
    private static final double DEGREES_PER_TICK = 360.0 / 4096;
    private final WPI_TalonSRX turretTalon;
    private final Provider<TurretControl> command;
    private final NavXGyro gyro;
    private final VisionUtil vision;
    private static boolean isAutoAimEnabled;
    private final RobotShuffleboardTab tab;
    public static boolean isAbsoluteAngle;


    public Turret(int id, Provider<TurretControl> command, NavXGyro gyro, VisionUtil vision, RobotShuffleboard shuffleboard) {
        this.command = command;
        this.gyro = gyro;
        this.vision = vision;
        this.tab = shuffleboard.getTab("TurretHood");
        turretTalon = new WPI_TalonSRX(id);
        turretTalon.configFactoryDefault();
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition,
                new PIDConfig((1.5 * 1023 / 400), 0, 0, 0), 800, turretTalon);
        turretTalon.setSensorPhase(false);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        turretTalon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    }

    @Override
    public double getPositionTicks() {
        return turretTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getAbsoluteAngle() {
        return MathUtil.wrapToCircle((getPositionTicks() - ZERO_TICKS) * DEGREES_PER_TICK + (gyro.getYaw()));
    }

    public void enableAutoAim() {
        isAutoAimEnabled = true;
    }

    public void disableAutoAim() {
        isAutoAimEnabled = false;
    }

    public double getAngle() {
        if(isAbsoluteAngle) {
            return MathUtil.wrapToCircle(gyro.getYaw() + ((getPositionTicks() - ZERO_TICKS) * DEGREES_PER_TICK));
        } else {
            return MathUtil.wrapToCircle((getPositionTicks() - ZERO_TICKS) * DEGREES_PER_TICK);
        }

    }

    public double getTxTurret(double distance, double extraDistance) {
        double tx = Math.toRadians(vision.getLLValue("tx"));
        double txTurret = Math.atan2(distance * Math.sin(tx) + ShooterConstants.getLLtoTurretZ(), distance * Math.cos(tx) + extraDistance);
        tab.setEntry("txTurret", txTurret);
        return txTurret;
    }

    public void setPositionTicks(double positionTicks) {
        double ticks = MathUtil.limit(
                ZERO_TICKS + positionTicks, ZERO_TICKS + getMinAngleInTicks(), ZERO_TICKS + getMaxAngleInTicks());
        tab.setEntry("PosTicks", ticks);
        turretTalon.set(ControlMode.MotionMagic, ticks);
    }

    public void setAngle(double angle) {
        angle = MathUtil.wrapToCircle(angle);
        if(isAbsoluteAngle){
            angle += MathUtil.wrapToCircle(gyro.getYaw());
        }

        tab.setEntry("TurretSetAngle", angle);

        double ticks = ZERO_TICKS + (angle * TICKS_PER_DEGREE);
        ticks = MathUtil.limit(ticks, ZERO_TICKS + getMinAngleInTicks(), ZERO_TICKS + getMaxAngleInTicks());

        tab.setEntry("TurretSetTick", ticks);

        turretTalon.set(ControlMode.MotionMagic, ticks);
    }

    public double getMaxAngleInTicks() {
        return MAX_ANGLE * TICKS_PER_DEGREE;
    }

    public double getMinAngleInTicks() {
        return MIN_ANGLE * TICKS_PER_DEGREE;
    }

    public double getAnglesOrInchesToTicks() {
        return 4096/360.0;
    }

    @Override
    public void setPower(double pwr) {
        if (pwr <= 0 && getPositionTicks() - ZERO_TICKS < getMinAngleInTicks()) {
            pwr = 0;
        } else if (pwr > 0 && getPositionTicks() - ZERO_TICKS > getMaxAngleInTicks()) {
            pwr = 0;
        }
        turretTalon.set(ControlMode.PercentOutput, pwr);
    }

    /*
    @Override
    public void setManualPower(double pwr) {
        if (pwr <= 0 && getPositionTicks() - zeroTicks < getMinAngleInTicks()) {
            pwr = 0;
        } else if (pwr > 0 && getPositionTicks() - zeroTicks > getMaxAngleInTicks()) {
            pwr = 0;
        }
        super.setManualPower(pwr);
    }
    */

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }

    public boolean isAutoAimEnabled() {
        return isAutoAimEnabled;
    }
}