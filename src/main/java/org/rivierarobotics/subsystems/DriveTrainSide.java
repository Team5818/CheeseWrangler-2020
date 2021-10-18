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

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;
import org.rivierarobotics.util.MotorUtil;

/**
 * Component part of drive train. Each side has three motors driven with the
 * same inversion status. A shaft encoder is present on each side (external)
 * to keep track of position, compensated for by gear ratio calculations.
 * Contains a velocity PIDF loop for PathTracer controlled per-motor.
 *
 * @see DriveTrain
 */
public class DriveTrainSide implements RRSubsystem {
    private static final double TICKS_PER_METER = 4280;
    private static final double MOTOR_TO_WHEEL_RATIO = (1.0 / 3) * (17.0 / 48);
    private final WPI_TalonFX mainLeft;
    private final WPI_TalonFX secondaryRight;
    private final WPI_TalonFX secondaryTop;
    private final Encoder shaftEncoder;
    private final MechLogger logger;
    private final boolean invert;

    public DriveTrainSide(DTMotorIds motors, boolean invert) {
        this.mainLeft = new WPI_TalonFX(motors.main);
        this.secondaryRight = new WPI_TalonFX(motors.secondaryOne);
        this.secondaryTop = new WPI_TalonFX(motors.secondaryTwo);
        this.mainLeft.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0.1));
        this.secondaryRight.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0.1));
        this.secondaryTop.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0.1));

        this.logger = Logging.getLogger(getClass(), invert ? "left" : "right");
        this.invert = invert;
        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor,
            new PIDConfig(0.22, 0, 0, 0.051), 0,
                mainLeft, secondaryRight, secondaryTop);
        mainLeft.setInverted(invert);
        secondaryRight.setInverted(invert);
        secondaryTop.setInverted(invert);
        mainLeft.setNeutralMode(NeutralMode.Brake);
        secondaryRight.setNeutralMode(NeutralMode.Brake);
        secondaryTop.setNeutralMode(NeutralMode.Brake);

        this.shaftEncoder = new Encoder(motors.encoderA, motors.encoderB);
        shaftEncoder.setReverseDirection(true);
        shaftEncoder.setDistancePerPulse(1 / TICKS_PER_METER);
    }

    @Override
    public double getPositionTicks() {
        return getPosition() * TICKS_PER_METER;
    }

    @Override
    public void setPower(double pwr) {
        logger.powerChange(pwr);
        mainLeft.set(TalonFXControlMode.PercentOutput, pwr);
        secondaryRight.set(TalonFXControlMode.PercentOutput, pwr);
        secondaryTop.set(TalonFXControlMode.PercentOutput, pwr);
    }

    public double getPosition() {
        return shaftEncoder.getDistance();
    }

    public double getVelocity() {
        return shaftEncoder.getRate();
    }

    /**
     * Sets the velocity of each motor individually with a PIDF loop.
     * Converts from m/s to ticks/100ms internally (set units).
     *
     * @param vel the velocity in meters per second to set.
     */
    public void setVelocity(double vel) {
        double set = vel * TICKS_PER_METER / MOTOR_TO_WHEEL_RATIO / 10;
        logger.setpointChange(set);
        mainLeft.set(ControlMode.Velocity, set);
        secondaryRight.set(ControlMode.Velocity, set);
        secondaryTop.set(ControlMode.Velocity, set);
    }

    public void setVoltage(double volts) {
        mainLeft.setVoltage(volts);
        secondaryRight.setVoltage(volts);
        secondaryTop.setVoltage(volts);
    }

    public void resetEncoder() {
        shaftEncoder.reset();
    }

    public MotorTemp[] getTemps() {
        final String side = (invert ? "Left" : "Right") + ": ";
        MotorTemp[] temps = new MotorTemp[3];
        temps[0] = new MotorTemp(mainLeft.getDeviceID(), mainLeft.getTemperature(), side + "mainLeft");
        temps[1] = new MotorTemp(secondaryRight.getDeviceID(), secondaryRight.getTemperature(), side + "secondaryRight");
        temps[2] = new MotorTemp(secondaryTop.getDeviceID(), secondaryTop.getTemperature(), side + "secondaryTop");
        return temps;
    }
}
