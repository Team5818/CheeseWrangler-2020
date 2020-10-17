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


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;
import org.rivierarobotics.commands.turret.TurretControl;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.MotorUtil;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;
import org.rivierarobotics.util.ShooterConstants;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Provider;

public class Turret extends SubsystemBase implements RRSubsystem {
    private static final double ZERO_TICKS = 1652;
    private static final double MAX_ANGLE = 10;
    private static final double MIN_ANGLE = -220;
    private final WPI_TalonSRX turretTalon;
    private final Provider<TurretControl> command;
    private final NavXGyro gyro;
    private final VisionUtil vision;
    private final RobotShuffleboardTab tab;
    private final MechLogger logger;

    public Turret(int id, Provider<TurretControl> command, NavXGyro gyro, VisionUtil vision, RobotShuffleboard shuffleboard) {
        this.command = command;
        this.gyro = gyro;
        this.vision = vision;
        this.tab = shuffleboard.getTab("TurretHood");
        this.logger = Logging.getLogger(getClass());

        this.turretTalon = new WPI_TalonSRX(id);
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition,
                new PIDConfig((1.5 * 1023 / 400), 0, 0, 0), 800, turretTalon);
        turretTalon.setSensorPhase(false);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        turretTalon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        turretTalon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        turretTalon.configForwardSoftLimitThreshold((int)(ZERO_TICKS + MathUtil.degreesToTicks(MAX_ANGLE)));
        turretTalon.configReverseSoftLimitThreshold((int)(ZERO_TICKS + MathUtil.degreesToTicks(MIN_ANGLE)));
        turretTalon.configForwardSoftLimitEnable(true);
        turretTalon.configReverseSoftLimitEnable(true);
    }

    @Override
    public double getPositionTicks() {
        return turretTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getVelocity() {
        return turretTalon.getSensorCollection().getPulseWidthVelocity();
    }

    public double getAngle(boolean isAbsolute) {
        return MathUtil.wrapToCircle(isAbsolute ? gyro.getYaw() + MathUtil.degreesToTicks(getPositionTicks() - ZERO_TICKS) :
                MathUtil.ticksToDegrees(getPositionTicks() - ZERO_TICKS));
    }

    public double getTxTurret(double extraDistance, double hoodAngle) {
        double tx = Math.toRadians(vision.getLLValue("tx"));
        double distance = ShooterConstants.getTopHeight() + ShooterConstants.getLLtoTurretY()
                / Math.tan(Math.toRadians(vision.getActualTY(hoodAngle)));
        double txTurret = Math.atan2(distance * Math.sin(tx) - ShooterConstants.getLLtoTurretZ(), distance * Math.cos(tx) + extraDistance);
        tab.setEntry("txTurret", txTurret);
        return txTurret;
    }

    public void setPositionTicks(double positionTicks) {
        tab.setEntry("TurretSetPosTicks", positionTicks);
        turretTalon.set(ControlMode.MotionMagic, positionTicks);
    }

    public void setAngle(double angle, boolean isAbsolute) {
        tab.setEntry("TSetAngle", angle);
        angle %= 360;
        if (isAbsolute) {
            angle -= MathUtil.wrapToCircle(gyro.getYaw());
        }
        double min = ZERO_TICKS + MathUtil.ticksToDegrees(MIN_ANGLE);
        double max = ZERO_TICKS + MathUtil.ticksToDegrees(MAX_ANGLE);
        double initialTicks = ZERO_TICKS + (MathUtil.degreesToTicks(angle) % 4096);
        tab.setEntry("TInitSetAngle", initialTicks);
        double ticks = MathUtil.limit(initialTicks, min, max);
        if (ticks == max) {
            initialTicks -= 4096;
        } else if (ticks == min) {
            initialTicks += 4096;
        }
        initialTicks = MathUtil.limit(initialTicks, min, max);
        if (initialTicks == max || initialTicks == min) {
            initialTicks = ticks;
        }
        tab.setEntry("TFinalAngleTicks", initialTicks);
        logger.setpointChange(initialTicks);
        turretTalon.set(ControlMode.MotionMagic, initialTicks);
    }

    public void setVelocity(double ticksPerSec) {
        ticksPerSec = ensureInRange(ticksPerSec);
        logger.stateChange("Velocity Set", ticksPerSec);
        turretTalon.set(ControlMode.Velocity, ticksPerSec);
    }

    @Override
    public void setPower(double pwr) {
        pwr = ensureInRange(pwr);
        logger.powerChange(pwr);
        turretTalon.set(ControlMode.PercentOutput, pwr);
    }

    private double ensureInRange(double in) {
        double pos = getPositionTicks() - ZERO_TICKS;
        if ((in < 0 && pos < MathUtil.ticksToDegrees(MIN_ANGLE))
                || in > 0 && pos > MathUtil.ticksToDegrees(MAX_ANGLE)) {
            in = 0;
        }
        return in;
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}