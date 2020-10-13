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
import org.rivierarobotics.commands.hood.HoodControl;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.MotorUtil;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;

import javax.inject.Provider;

public class Hood extends SubsystemBase implements RRSubsystem {
    private static final double ZERO_TICKS = 2750;
    private static final double FORWARD_TICKS = 2500;
    private static final double BACK_TICKS = 2200;
    private final WPI_TalonSRX hoodTalon;
    private final Provider<HoodControl> command;
    private final MechLogger logger;
    private final RobotShuffleboardTab shuffleTab;

    public Hood(int motorId, Provider<HoodControl> command, RobotShuffleboard shuffleboard) {
        this.command = command;
        this.logger = Logging.getLogger(getClass());
        this.shuffleTab = shuffleboard.getTab("TurretHood");

        this.hoodTalon = new WPI_TalonSRX(motorId);
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition,
            new PIDConfig((0.8 * 1023 / 300), 0, 0, 0), 600, hoodTalon);
        hoodTalon.setSensorPhase(true);
        hoodTalon.setInverted(true);
        hoodTalon.setNeutralMode(NeutralMode.Brake);
        hoodTalon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        hoodTalon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        hoodTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    }

    public double getZeroTicks() {
        return ZERO_TICKS;
    }
    
    public double getForwardTicks() {
        return FORWARD_TICKS;
    }

    public double getBackTicks() {
        return BACK_TICKS;
    }

    public double getPositionTicks() {
        return hoodTalon.getSensorCollection().getPulseWidthPosition();
    }

    public void setPower(double pwr) {
        pwr = curvePower(pwr);
        logger.powerChange(pwr);
        hoodTalon.set(ControlMode.PercentOutput, pwr);
    }

    // Applies a bell curve power ramp for safety
    private double curvePower(double pwr) {
        double range = FORWARD_TICKS - BACK_TICKS;
        double pos = 1.5 - (pwr >= 0 ? 1.5 / (range / Math.abs((FORWARD_TICKS - getPositionTicks()))) :
                1.5 / (range / Math.abs((getPositionTicks() - BACK_TICKS))));
        shuffleTab.setEntry("curvePos", pos);
        double curvedPwr = Math.pow(Math.E, -(8.0 / 8) * pos * pos) * pwr;
        boolean limitSafety = limitSafety(pwr);
        shuffleTab.setEntry("limitSafety", limitSafety);
        shuffleTab.setEntry("curvedPwr", curvedPwr);
        return limitSafety ? curvedPwr : 0.0;
    }

    private boolean limitSafety(double pwr) {
        double limit = MathUtil.limit(getPositionTicks(), BACK_TICKS, FORWARD_TICKS);
        if ((limit != FORWARD_TICKS && limit != BACK_TICKS) || (limit == FORWARD_TICKS && pwr < 0)) {
            return true;
        }
        return limit == BACK_TICKS && pwr > 0;
    }

    public double getAngle() {
        return MathUtil.ticksToDegrees(ZERO_TICKS - getPositionTicks());
    }

    public void setAngle(double angle) {
        shuffleTab.setEntry("setAngle", angle);
        double ticks = MathUtil.limit(ZERO_TICKS - MathUtil.degreesToTicks(angle), BACK_TICKS, FORWARD_TICKS);
        shuffleTab.setEntry("setTicks", ticks);
        logger.setpointChange(ticks);
        hoodTalon.set(ControlMode.MotionMagic, ticks);
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
