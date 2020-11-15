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
import org.rivierarobotics.util.MotorUtil;

public class Climb extends SubsystemBase implements RRSubsystem {
    //private final WPI_TalonSRX climbTalon;
    //private final MechLogger logger;

    public Climb(int id) {
        //climbTalon = new WPI_TalonSRX(id);
        //MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition,
        //        new PIDConfig((1023 * 0.1) / 500, 0, 0, (1023.0 * 0.75) / 15900), 0, climbTalon);
        //logger = Logging.getLogger(getClass());
        //climbTalon.setSensorPhase(true);
        //climbTalon.setNeutralMode(NeutralMode.Brake);
    }

    public void setPositionTicks(double position) {
        //climbTalon.set(ControlMode.MotionMagic, position);
    }

    @Override
    public double getPositionTicks() {
        //return climbTalon.getSelectedSensorPosition();
        throw new UnsupportedOperationException();
    }

    @Override
    public void setPower(double pwr) {
        //logger.powerChange(pwr);
        //climbTalon.set(ControlMode.PercentOutput, pwr);
    }
}
