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
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;
import org.rivierarobotics.commands.turret.TurretControl;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.MotorUtil;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;
import org.rivierarobotics.util.ShooterConstants;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Provider;

public class Turret extends SubsystemBase implements RRSubsystem {
    private static final double ZERO_TICKS = 1652;
    private static final double MAX_ANGLE = 10;
    private static final double MIN_ANGLE = -50;
    private static final int FORWARD_LIMIT_TICKS = (int) (ZERO_TICKS + MathUtil.degreesToTicks(MAX_ANGLE));
    private static final int BACK_LIMIT_TICKS = (int) (ZERO_TICKS + MathUtil.degreesToTicks(MIN_ANGLE));
    private final WPI_TalonSRX turretTalon;
    private final Provider<TurretControl> command;
    private final NavXGyro gyro;
    private final VisionUtil vision;
    private final RobotShuffleboardTab tab;
    private final MechLogger logger;

    public Turret(int id, Provider<TurretControl> command, NavXGyro gyro, VisionUtil vision, RobotShuffleboard shuffleboard) {
        this.command = command;
        this.gyro = gyro;
        this.vision = vision;
        this.tab = shuffleboard.getTab("TurretHood");
        this.logger = Logging.getLogger(getClass());

        this.turretTalon = new WPI_TalonSRX(id);
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition,
                PIDMode.POSITION.config, 800, turretTalon);
        turretTalon.setSensorPhase(false);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        MotorUtil.setSoftLimits(FORWARD_LIMIT_TICKS, BACK_LIMIT_TICKS, turretTalon);
    }

    public int getForwardLimit() {
        return FORWARD_LIMIT_TICKS;
    }

    public int getBackLimit() {
        return BACK_LIMIT_TICKS;
    }

    @Override
    public double getPositionTicks() {
        return turretTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getVelocity() {
        return turretTalon.getSensorCollection().getPulseWidthVelocity();
    }

    public double getAngle(boolean isAbsolute) {
        return isAbsolute ? MathUtil.wrapToCircle(gyro.getYaw() + MathUtil.ticksToDegrees(getPositionTicks() - ZERO_TICKS)) :
                MathUtil.ticksToDegrees(getPositionTicks() - ZERO_TICKS);
    }

    public double[] getTurretCalculations(double extraDistance, double hoodAngle) {
        double initialD = ShooterConstants.getTopHeight() / Math.sin(Math.toRadians(vision.getActualTY(hoodAngle)));
        double tx = Math.toRadians(vision.getLLValue("tx"));
        double xInitialD = Math.sin(tx) * initialD;
        double yInitialD = Math.cos(tx) * initialD + extraDistance;
        double dist = Math.sqrt(xInitialD * xInitialD + yInitialD * yInitialD);
        tab.setEntry("dist", dist);
        tx = Math.atan(xInitialD / yInitialD);
        tab.setEntry("l", Math.toDegrees(tx));
        double angleA = Math.PI / 2 - tx;
        double z = ShooterConstants.getLLtoTurretZ();
        double a = Math.sqrt(dist * dist + z * z - 2 * dist * z * Math.cos(angleA));
        tab.setEntry("a", a);
        double finalAngle = Math.toDegrees(Math.asin((Math.sin(angleA) * dist / a)));
        tab.setEntry("finalAngle", finalAngle);
        tab.setEntry("thing", Math.toDegrees(Math.asin(z / dist)));
        finalAngle = tx > Math.asin(z / dist) ? 90 - finalAngle : finalAngle - 90;
        tab.setEntry("finalAngle2", finalAngle + getAngle(true));
        return new double[] { a, MathUtil.wrapToCircle(finalAngle + getAngle(true)) };

    }

    public void setPositionTicks(double positionTicks) {
        tab.setEntry("TurretSetPosTicks", positionTicks);
        PIDMode.POSITION.enable(turretTalon);
        turretTalon.set(ControlMode.MotionMagic, positionTicks);
    }

    public void setAngle(double angle, boolean isAbsolute) {
        tab.setEntry("TSetAngle", angle);
        angle %= 360;
        if (isAbsolute) {
            angle -= MathUtil.wrapToCircle(gyro.getYaw());
        }
        double min = ZERO_TICKS + MathUtil.degreesToTicks(MIN_ANGLE);
        double max = ZERO_TICKS + MathUtil.degreesToTicks(MAX_ANGLE);
        double initialTicks = ZERO_TICKS + (MathUtil.degreesToTicks(angle) % 4096);
        double ticks = MathUtil.limit(initialTicks, min, max);
        if (ticks == max) {
            initialTicks -= 4096;
        } else if (ticks == min) {
            initialTicks += 4096;
        }
        initialTicks = MathUtil.limit(initialTicks, min, max);
        if (initialTicks == max || initialTicks == min) {
            initialTicks = ticks;
        }
        tab.setEntry("TFinalAngleTicks", initialTicks);
        logger.setpointChange(initialTicks);
        PIDMode.POSITION.enable(turretTalon);
        turretTalon.set(ControlMode.MotionMagic, initialTicks);
    }

    public void setVelocity(double ticksPer100ms) {
        logger.stateChange("Velocity Set", ticksPer100ms);
        PIDMode.VELOCITY.enable(turretTalon);
        turretTalon.set(ControlMode.Velocity, ticksPer100ms);
    }

    @Override
    public void setPower(double pwr) {
        logger.powerChange(pwr);
        turretTalon.set(ControlMode.PercentOutput, pwr);
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }

    private enum PIDMode {
        POSITION(new PIDConfig((1.5 * 1023 / 400), 0, 0, 0)),
        VELOCITY(new PIDConfig((1.5 * 1023 / 400), 0, 0, 0));

        private final PIDConfig config;
        private static PIDMode current;

        PIDMode(PIDConfig config) {
            this.config = config;
        }

        public void enable(BaseMotorController motor) {
            if (this != current) {
                motor.config_kP(0, config.getP());
                motor.config_kI(0, config.getI());
                motor.config_kD(0, config.getD());
                motor.config_kF(0, config.getF());
                current = this;
            }
        }
    }
}