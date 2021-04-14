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
    private static final int ZERO_TICKS = 2800;
    private static final int FORWARD_LIMIT_TICKS = 2550;
    private static final int BACK_LIMIT_TICKS = 2200;
    private static final double CURVE_FACTOR = 1.5;
    private final WPI_TalonSRX hoodTalon;
    private final Provider<HoodControl> command;
    private final RobotShuffleboardTab tab;
    private final MultiPID multiPID;
    private final MechLogger logger;

    public Hood(int motorId, Provider<HoodControl> command, RobotShuffleboard shuffleboard) {
        this.command = command;
        this.logger = Logging.getLogger(getClass());
        this.tab = shuffleboard.getTab("TurretHood");

        this.hoodTalon = new WPI_TalonSRX(motorId);
        this.multiPID = new MultiPID(hoodTalon,
                new PIDConfig((0.8 * 1023 / 300), 0, 0, 0),
                new PIDConfig(0, 0, 0, 0));
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition,
                multiPID.getConfig(MultiPID.Type.POSITION), 600, hoodTalon);
        multiPID.getConfig(MultiPID.Type.VELOCITY).applyTo(hoodTalon, 1);
        multiPID.getConfig(MultiPID.Type.POSITION).applyTo(hoodTalon, 0);
        MotorUtil.setSoftLimits(FORWARD_LIMIT_TICKS, BACK_LIMIT_TICKS, hoodTalon);
        hoodTalon.setSensorPhase(true);
        hoodTalon.setInverted(true);
        hoodTalon.setNeutralMode(NeutralMode.Brake);
    }

    public int getZeroTicks() {
        return ZERO_TICKS;
    }
    
    public int getForwardLimit() {
        return FORWARD_LIMIT_TICKS;
    }

    public int getBackLimit() {
        return BACK_LIMIT_TICKS;
    }

    public double getPositionTicks() {
        return hoodTalon.getSelectedSensorPosition();
    }

    public void setPower(double pwr) {
        double range = FORWARD_LIMIT_TICKS - BACK_LIMIT_TICKS;
        double pos = CURVE_FACTOR - (pwr >= 0 ? CURVE_FACTOR / (range / Math.abs((FORWARD_LIMIT_TICKS - getPositionTicks()))) :
                CURVE_FACTOR / (range / Math.abs((getPositionTicks() - BACK_LIMIT_TICKS))));
        pwr = Math.pow(Math.E, -pos * pos) * pwr;
        logger.powerChange(pwr);
        hoodTalon.set(ControlMode.PercentOutput, pwr);
    }

    public double getAngle() {
        return MathUtil.ticksToDegrees(ZERO_TICKS - getPositionTicks());
    }

    public double getZeroedAngle(double toZero) {
        return MathUtil.ticksToDegrees(ZERO_TICKS - toZero);
    }

    public void setAngle(double angle) {
        double ticks = ZERO_TICKS - MathUtil.degreesToTicks(angle);
        tab.setEntry("setAngle", angle);
        tab.setEntry("setTicks", ticks);
        logger.setpointChange(ticks);
        multiPID.selectConfig(MultiPID.Type.POSITION);
        hoodTalon.set(ControlMode.MotionMagic, ticks);
    }

    public void setVelocity(double vel) {
        tab.setEntry("setVel", vel);
        multiPID.selectConfig(MultiPID.Type.VELOCITY);
        hoodTalon.set(ControlMode.Velocity, vel);
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
