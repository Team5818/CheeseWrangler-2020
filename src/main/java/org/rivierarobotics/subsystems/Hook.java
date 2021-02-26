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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.climb.HookControl;
import org.rivierarobotics.util.MotorUtil;

import javax.inject.Provider;

public class Hook extends SubsystemBase implements RRSubsystem {
    private final WPI_TalonFX hookTalon;
    private final Provider<HookControl> command;

    public Hook(int motorID, Provider<HookControl> command) {
        this.hookTalon = new WPI_TalonFX(motorID);
        this.command = command;
        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor,
                new PIDConfig(0, 0, 0, 0), 0, hookTalon);
        hookTalon.setSensorPhase(true);
        hookTalon.setNeutralMode(NeutralMode.Brake);
    }

    public void setPositionTicks(double positionTicks) {
        hookTalon.set(ControlMode.MotionMagic, positionTicks);
    }

    @Override
    public double getPositionTicks() {
        return hookTalon.getSelectedSensorPosition();
    }

    @Override
    public void setPower(double pwr) {
        hookTalon.set(pwr);
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
