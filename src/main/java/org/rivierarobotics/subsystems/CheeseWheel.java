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
import org.rivierarobotics.commands.cheesewheel.CheeseWheelControl;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.RobotShuffleboard;

import javax.inject.Provider;

public class CheeseWheel extends BasePIDSubsystem implements RRSubsystem {
    private final WPI_TalonSRX wheelTalon;
    private final Provider<CheeseWheelControl> command;
    private final RobotShuffleboard shuffleboard;
    private final AnalogInput frontSensor;
    private final AnalogInput backSensor;
    private final double zeroTicks = 3725;
    private final double ballMax = 260;
    private final double ballMin = 100;

    public CheeseWheel(int motor, int frontSensor, int backSensor, Provider<CheeseWheelControl> command, RobotShuffleboard shuffleboard) {
        super(new PIDConfig(0.002, 0.0, 0.0001, 0.0, 30, 1.0));
        this.wheelTalon = new WPI_TalonSRX(motor);
        this.frontSensor = new AnalogInput(frontSensor);
        this.backSensor = new AnalogInput(backSensor);
        this.command = command;
        this.shuffleboard = shuffleboard;
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void setPower(double pwr) {
        wheelTalon.set(pwr);
    }

    @Override
    public double getPositionTicks() {
        return wheelTalon.getSensorCollection().getPulseWidthPosition();
    }

    public int getIndex(AngleOffset mode) {
        double cwAngle = getAdjustedAngle(getAngleOffset(mode));
        int min = 360;
        double minAngle = 360;
        for (int i = 0; i < 6; i++) {
            double ang = Math.abs(cwAngle - (i * 72));
            if (minAngle > ang) {
                minAngle = ang;
                if (i == 5) {
                    return 0;
                }
                min = i;
            }
        }
        logger.stateChange("slot_index", min);
        return min;
    }

    public double getAngle() {
        return getAdjustedAngle(0);
    }

    public double getAdjustedAngle(double adjAngle) {
        return MathUtil.wrapToCircle((getPositionTicks() - zeroTicks) / getAnglesOrInchesToTicks() + adjAngle);
    }

    public double getAngleAdded(int index, AngleOffset mode, int direction) {
        shuffleboard.getTab("Cheese Wheel").setEntry("Set Index", index);
        double angleOff = index * 72 - getAdjustedAngle(getAngleOffset(mode));
        if (Math.abs(angleOff) > 180) {
            if (angleOff < 0) {
                angleOff += 360;
            } else if (angleOff > 0) {
                angleOff -= 360;
            }
        }

        if (direction > 0) {
            if (angleOff < 0) {
                return angleOff + 360;
            }
        } else if (direction < 0) {
            if (angleOff > 0) {
                return angleOff - 360;
            }
        }

        return angleOff;
    }

    public void addAngle(double angle) {
        setPositionTicks(getPositionTicks() + angle * getAnglesOrInchesToTicks());
    }

    public boolean isFrontBallPresent() {
        boolean frontBall = frontSensor.getValue() < ballMax && frontSensor.getValue() > ballMin;
        logger.stateChange("front_ball_present", frontBall);
        return frontBall;
    }

    public double getFrontSensorValue() {
        return frontSensor.getValue();
    }

    public boolean isBackBallPresent() {
        boolean backBall = backSensor.getValue() < ballMax && backSensor.getValue() > ballMin;
        logger.stateChange("back_ball_present", backBall);
        return backBall;
    }

    public double getBackSensorValue() {
        return backSensor.getValue();
    }

    public double getAngleOffset(AngleOffset offset) {
        return offset.angle;
    }

    public enum AngleOffset {
        SHOOTING(36), COLLECT_FRONT(106), COLLECT_BACK(253);

        public final int angle;
        AngleOffset(int angle) {
            this.angle = angle;
        }
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
