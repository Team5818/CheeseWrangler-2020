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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelControl;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Provider;

public class CheeseWheel extends BasePIDSubsystem {
    private final WPI_TalonSRX wheelTalon;
    private final Provider<CheeseWheelControl> command;
    private static final double INPUT_RANGE = 4095;
    private final AnalogInput frontSensor;
    private final DigitalOutput backSensor;
    private final double zeroTicks = 3725;
    public Mode lastMode = Mode.COLLECT_FRONT;
    public boolean isRunning;

    public CheeseWheel(int motor, int frontSensor, int backSensor, Provider<CheeseWheelControl> command) {
        super(new PIDConfig(0.002, 0.0, 0.0001, 0.0, 30, 1.0));
        this.wheelTalon = new WPI_TalonSRX(motor);
        this.frontSensor = new AnalogInput(frontSensor);
        this.backSensor = new DigitalOutput(9);
        this.command = command;
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
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
        int base = wheelTalon.getSensorCollection().getPulseWidthPosition();
        return base;
    }

    public double getClosestIndexAngle(Mode mode, Filled filled, int direction) {
        return getClosestSlot(mode, filled, direction).getModePosition(mode);
    }

    public CheeseSlot getClosestSlot(Mode mode, Filled filled, int direction) {
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
            if (0 != direction && ((int) Math.signum(diff)) != direction) {
                // only take those with same direction
                continue;
            }
            diff = Math.abs(diff);
            if (diff < minDiff) {
                minSlot = allSlot;
                minDiff = diff;
            }
        }
        return minSlot;
    }

    public int getIndex(AngleOffset mode) {
        double cwAngle = getAdjustedAngle(getAngleOffset(mode));
        int min = 360;
        double minAngle = 360;
        for (int i = 0; i < 6; i++) {
            if (minAngle > Math.abs(cwAngle - (i * 72))) {
                minAngle = Math.abs(cwAngle - (i * 72));
                if (i == 5) {
                    return 0;
                }
                min = i;
            }
        }
        return min;
    }

    public boolean CommandIsRunning() {
        return isRunning;
    }

    public double getAdjustedAngle(double adjAngle) {
        return MathUtil.wrapToCircle((getPositionTicks() - zeroTicks) / getAnglesOrInchesToTicks() + adjAngle);
    }

    public double getAngleAdded(int index, AngleOffset mode, int direction) {
        SmartDashboard.putNumber("setIndex", index);
        double angleOff = index * 72 - getAdjustedAngle(getAngleOffset(mode));
        if (Math.abs(angleOff) > 180) {
            if(angleOff < 0) {
                angleOff += 360;
            } else if(angleOff > 0) {
                angleOff -= 360;
            }
        }

        if(direction > 0) {
            if(angleOff < 0) {
                return angleOff + 360;
            }
        } else if (direction < 0) {
            if(angleOff > 0) {
                return angleOff - 360;
            }
        }



        return angleOff;
    }


    public void addAngle(double angle) {
        SmartDashboard.putNumber("CWSetPosition", wheelTalon.getSensorCollection().getPulseWidthPosition() + angle * getAnglesOrInchesToTicks());
        isRunning = true;
        setPositionTicks(wheelTalon.getSensorCollection().getPulseWidthPosition() + angle * getAnglesOrInchesToTicks());
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


    public boolean isFrontBallPresent() {
        return (frontSensor.getValue() < 260 && frontSensor.getValue() > 100);
    }

    public double getFrontSensorValue() {
        return frontSensor.getValue();
    }

    public boolean isBackBallPresent() {
        return true;
    }

    public boolean getBackSensorValue() {
        return backSensor.get();
    }

    public enum Filled {
        YES, NO, DONT_CARE
    }

    public enum Mode {
        COLLECT_FRONT, COLLECT_BACK, FIX_FRONT, FIX_BACK, SHOOTING
    }

    public enum AngleOffset {
        SHOOTING, COLLECT_FRONT, COLLECT_BACK
    }

    public double getAngleOffset(AngleOffset offset) {
        if(offset.equals(AngleOffset.SHOOTING)) {
            return 36;
        } else if(offset.equals(AngleOffset.COLLECT_FRONT)) {
            return 106;
        } else if(offset.equals(AngleOffset.COLLECT_BACK)) {
            return 253;
        } else {
            return 0;
        }
    }

}
