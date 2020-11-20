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
    private static final double ZERO_TICKS = 630;
    private static final double MAX_ANGLE = 10;
    private static final double MIN_ANGLE = -200;
    private static final int FORWARD_LIMIT_TICKS = (int) (ZERO_TICKS + MathUtil.degreesToTicks(MAX_ANGLE));
    private static final int BACK_LIMIT_TICKS = (int) (ZERO_TICKS + MathUtil.degreesToTicks(MIN_ANGLE));
    private final WPI_TalonSRX turretTalon;
    private final Provider<TurretControl> command;
    private final NavXGyro gyro;
    private final VisionUtil vision;
    private final RobotShuffleboardTab tab;
    private final MultiPID multiPID;
    private final MechLogger logger;

    public Turret(int id, Provider<TurretControl> command, NavXGyro gyro, VisionUtil vision, RobotShuffleboard shuffleboard) {
        this.command = command;
        this.gyro = gyro;
        this.vision = vision;
        this.tab = shuffleboard.getTab("TurretHood");
        this.logger = Logging.getLogger(getClass());


        this.turretTalon = new WPI_TalonSRX(id);
        this.multiPID = new MultiPID(turretTalon,
                new PIDConfig((1.5 * 1023 / 400), 0, 0, 0),
                new PIDConfig(0.7 * 1023 / 400, 0, 1.5 * 1023 * 0.01 / 400, (1023.0 * 0.3) / 70));
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition,
                multiPID.getConfig(MultiPID.Type.POSITION), 800, turretTalon);
        MotorUtil.setSoftLimits(FORWARD_LIMIT_TICKS, BACK_LIMIT_TICKS, turretTalon);
        turretTalon.setSensorPhase(false);
        turretTalon.setNeutralMode(NeutralMode.Brake);
    }

    public int getForwardLimit() {
        return FORWARD_LIMIT_TICKS;
    }

    public int getBackLimit() {
        return BACK_LIMIT_TICKS;
    }

    @Override
    public double getPositionTicks() {
        return turretTalon.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return turretTalon.getSelectedSensorVelocity();
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
        tx = Math.atan(xInitialD / yInitialD);

        double angleA = Math.PI / 2 - tx;
        double z = ShooterConstants.getLLtoTurretZ();
        double a = Math.sqrt(dist * dist + z * z - 2 * dist * z * Math.cos(angleA));
        double finalAngle = Math.toDegrees(Math.asin((Math.sin(angleA) * dist / a)));

        finalAngle = tx > Math.asin(z / dist) ? 90 - finalAngle : finalAngle - 90;
        return new double[] { a, MathUtil.wrapToCircle(finalAngle + getAngle(true)) };
    }

    public void setPositionTicks(double positionTicks) {
        tab.setEntry("TurretSetPosTicks", positionTicks);
        multiPID.selectConfig(MultiPID.Type.POSITION);
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
            initialTicks = getPositionTicks() - ZERO_TICKS < 0 ? ticks : max;
        }
        tab.setEntry("TFinalAngleTicks", initialTicks);
        logger.setpointChange(initialTicks);
        multiPID.selectConfig(MultiPID.Type.POSITION);
        turretTalon.set(ControlMode.MotionMagic, initialTicks);
    }

    public void setVelocity(double ticksPer100ms) {
        logger.stateChange("Velocity Set", ticksPer100ms);
        tab.setEntry("setVelTicks", ticksPer100ms);
        multiPID.selectConfig(MultiPID.Type.VELOCITY);
        turretTalon.set(ControlMode.Velocity, ticksPer100ms);
        tab.setEntry("whatever", turretTalon.getClosedLoopTarget());
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
}