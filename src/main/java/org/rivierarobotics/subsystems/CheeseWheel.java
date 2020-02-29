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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.commands.CheeseWheelControl;
import org.rivierarobotics.util.CheeseSlot;

import javax.inject.Provider;

public class CheeseWheel extends BasePIDSubsystem {
    private final WPI_TalonSRX wheelTalon;
    private final Provider<CheeseWheelControl> command;
    private static final double INPUT_RANGE = 4095;
    private final CWSensors sensors;
    public Mode lastMode = Mode.COLLECT_FRONT;

    public CheeseWheel(int motor, CWSensors sensors, Provider<CheeseWheelControl> command) {
        super(new PIDConfig(0.0012, 0.0, 0, 0.1, 5, 0.5));
        this.wheelTalon = new WPI_TalonSRX(motor);
        this.command = command;
        this.sensors = sensors;
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
        pidController.enableContinuousInput(0, INPUT_RANGE);
    }

    @Override
    protected void setPower(double pwr) {
        wheelTalon.set(pwr);
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }

    @Override
    public double getPositionTicks() {
        int base = wheelTalon.getSensorCollection().getPulseWidthPosition() % 4096;
        while (base < 0) {
            base += 4096;
        }
        return base;
    }

    public double getClosestIndexAngle(Mode mode, Filled filled) {
        return getClosestSlot(mode, filled).getModePosition(mode);
    }

    public CheeseSlot getClosestSlot(Mode mode, Filled filled) {
        this.lastMode = mode;
        CheeseSlot[] allSlots = CheeseSlot.values();
        CheeseSlot minSlot = allSlots[0];
        double minDiff = Double.MAX_VALUE;

        for (CheeseSlot allSlot : allSlots) {
            if (filled != Filled.DONT_CARE && (filled == Filled.YES) != allSlot.isFilled) {
                continue;
            }

            double diff = allSlot.getModePosition(mode) - getPositionTicks();
            diff = correctDiffForGap(diff);
            diff = Math.abs(diff);
            if (diff < minDiff) {
                minSlot = allSlot;
                minDiff = diff;
            }
            SmartDashboard.putNumber("diff", diff);
        }
        return minSlot;
    }

    // Handles continuous input gap
    public double correctDiffForGap(double diff) {
        diff %= INPUT_RANGE;
        if (Math.abs(diff) > INPUT_RANGE / 2) {
            if (diff > 0) {
                return diff - INPUT_RANGE;
            } else {
                return diff + INPUT_RANGE;
            }
        }
        return diff;
    }

    public CWSensors getSensors() {
        return sensors;
    }

    public enum Filled {
        YES, NO, DONT_CARE
    }

    public enum Mode {
        COLLECT_FRONT, COLLECT_BACK, FIX_FRONT, FIX_BACK, SHOOTING
    }
}
