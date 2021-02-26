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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;
import org.rivierarobotics.commands.climb.ClimbControl;
import org.rivierarobotics.util.MotorUtil;

import javax.inject.Provider;


public class Climb extends SubsystemBase implements RRSubsystem {
    private static final double MAX_TICKS = 39114 * 35.93;
    private final WPI_TalonFX climbTalon;
    private final DigitalInput limit;
    private final Provider<ClimbControl> command;
    private final MechLogger logger;

    public Climb(int motorId, int limitId, Provider<ClimbControl> command) {
        this.climbTalon = new WPI_TalonFX(motorId);
        this.limit = new DigitalInput(limitId);
        this.command = command;
        this.logger = Logging.getLogger(getClass());

        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor,
                new PIDConfig(0, 0, 0, 0), 0, climbTalon);
        climbTalon.setSensorPhase(true);
        MotorUtil.setSoftLimits((int) MAX_TICKS, 0, climbTalon);
        climbTalon.setNeutralMode(NeutralMode.Brake);
    }

    public boolean isAtBottom() {
        return !limit.get();
    }

    public void resetEncoder() {
        climbTalon.setSelectedSensorPosition(0);
    }

    public void setPositionTicks(double position) {
        climbTalon.set(ControlMode.MotionMagic, position);
    }

    @Override
    public double getPositionTicks() {
        return climbTalon.getSelectedSensorPosition();
    }

    @Override
    public void setPower(double pwr) {
        logger.powerChange(pwr);
        climbTalon.set(ControlMode.PercentOutput, pwr);
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }

    public enum Position {
        ZERO, HALF, MAX;

        public double getTicks() {
            return this.ordinal() / 2.0 * MAX_TICKS;
        }
    }
}
