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
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.MotorUtil;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;

public class Flywheel extends SubsystemBase implements RRSubsystem {
    private final WPI_TalonFX flywheelFalcon;
    private final MechLogger logger;
    private final RobotShuffleboardTab tab;
    private static double targetVel = 0;

    public Flywheel(int id, RobotShuffleboard shuffleboard) {
        this.logger = Logging.getLogger(getClass());
        this.tab = shuffleboard.getTab("Vision");

        this.flywheelFalcon = new WPI_TalonFX(id);
        //new PIDConfig((1023.0 * 0.5) / 500, (1023.0 * 0.01) / 500, 0.0, (1023.0 * 0.75) / 15900), 0, flywheelFalcon);
        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor,
            new PIDConfig(1.5, 0.0, 0.3, (1023.0 * 0.72) / 15900), 0, flywheelFalcon);
        flywheelFalcon.setInverted(false);
        flywheelFalcon.setNeutralMode(NeutralMode.Coast);
        flywheelFalcon.configPeakOutputReverse(0);

    }

    @Override
    public double getPositionTicks() {
        return flywheelFalcon.getSelectedSensorVelocity();
    }

    @Override
    public void setPower(double pwr) {
        logger.powerChange(pwr);
        flywheelFalcon.set(ControlMode.PercentOutput, pwr);
    }

    public boolean withinTolerance(double tolerance) {
        return MathUtil.isWithinTolerance(getPositionTicks(), targetVel, tolerance);
    }

    public void setVelocity(double vel) {
        tab.setEntry("Flywheel Set Vel", vel);
        Flywheel.targetVel = vel;
        logger.setpointChange(vel);
        if (vel == 0) {
            flywheelFalcon.set(TalonFXControlMode.Velocity, 0.0);
            flywheelFalcon.set(TalonFXControlMode.Current, 0.0);
        } else {
            flywheelFalcon.set(TalonFXControlMode.Velocity, vel);
        }
    }
}
