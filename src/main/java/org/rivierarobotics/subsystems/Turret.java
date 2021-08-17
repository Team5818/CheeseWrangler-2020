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
import org.rivierarobotics.commands.turret.TurretControl;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.MotorUtil;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.RSTab;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.ShooterConstants;
import org.rivierarobotics.util.VisionUtil;

import javax.inject.Provider;

/**
 * Subsystem for the turret. Rotates on a limited range around Cheese Wheel
 * shoot opening to direct balls towards a target. Suggested use case is to
 * shoot in the same direction as Cheese Wheel travel. Has an issue with
 * wrapping around +/- 4096 ticks (full rotation), countered by wrapErr.
 * Contains a dual PID for position and velocity.
 */
public class Turret extends SubsystemBase implements RRSubsystem {
    private static final double ZERO_TICKS = 470;
    private static final double MAX_ANGLE = 7;
    private static final double MIN_ANGLE = -214;
    private static final int FORWARD_LIMIT_TICKS = (int) (ZERO_TICKS + MathUtil.degreesToTicks(MAX_ANGLE));
    private static final int BACK_LIMIT_TICKS = (int) (ZERO_TICKS + MathUtil.degreesToTicks(MIN_ANGLE));
    private static final int DETECT_BUFFER_TICKS = 100;
    private final WPI_TalonSRX turretTalon;
    private final Provider<TurretControl> command;
    private final NavXGyro gyro;
    private final VisionUtil vision;
    private final RSTab tab;
    private final MultiPID multiPID;
    private final MechLogger logger;
    private int wrapErrOffset;
    private long errLoopCtr;

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
        multiPID.applyAllConfigs();
        MotorUtil.setSoftLimits(FORWARD_LIMIT_TICKS, BACK_LIMIT_TICKS, turretTalon);
        turretTalon.setSensorPhase(false);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        checkWrapError();
    }

    public int getForwardLimit() {
        return FORWARD_LIMIT_TICKS;
    }

    public int getBackLimit() {
        return BACK_LIMIT_TICKS;
    }

    @Override
    public double getPositionTicks() {
        return turretTalon.getSelectedSensorPosition() + wrapErrOffset;
    }

    public double getVelocity() {
        return turretTalon.getSelectedSensorVelocity();
    }

    /**
     * Get the angle of the turret, where zero is straight forward (relative).
     *
     * @param isAbsolute if the angle should consider the robot heading.
     * @return the angle of the turret in degrees.
     */
    public double getAngle(boolean isAbsolute) {
        double relAngle = MathUtil.ticksToDegrees(getPositionTicks() - ZERO_TICKS);
        return isAbsolute ? MathUtil.wrapToCircle(gyro.getYaw() + relAngle) : relAngle;
    }

    /**
     * Calculate AutoAim distance and angle for the turret. Requires input of
     * hood angle for accurate shooting characteristics.
     *
     * @param extraDistance physics util extra shooting distance.
     * @param hoodAngle angle of the hood in degrees.
     * @return a double array containing a distance and angle.
     */
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
        double finalAngle = Math.toDegrees(Math.asin((Math.sin(angleA) * dist / a))) + 2;

        finalAngle = tx > Math.asin(z / dist) ? 90 - finalAngle : finalAngle - 90;
        return new double[] { a, MathUtil.wrapToCircle(finalAngle + getAngle(true)) };
    }

    /**
     * Set the position of the turret. Uses position PID loop configuration.
     *
     * @param positionTicks the position to set in ticks.
     */
    public void setPositionTicks(double positionTicks) {
        positionTicks -= wrapErrOffset;
        tab.setEntry("TurretSetPosTicks", positionTicks);
        multiPID.selectConfig(MultiPID.Type.POSITION);
        turretTalon.set(ControlMode.MotionMagic, positionTicks);
    }

    /**
     * Set the angle of the turret. Uses position PID loop configuration.
     * Wraps set value to be within safety bounds and take the shortest
     * distance/path. Will consider robot yaw if absolute
     *
     * @param angle the angle to set in degrees.
     * @param isAbsolute if the set angle should be absolute.
     */
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
        initialTicks -= wrapErrOffset;
        tab.setEntry("TFinalAngleTicks", initialTicks);
        logger.setpointChange(initialTicks);
        multiPID.selectConfig(MultiPID.Type.POSITION);
        turretTalon.set(ControlMode.MotionMagic, initialTicks);
    }

    /**
     * Set the velocity of the turret. Uses velocity PID loop configuration.
     *
     * @param ticksPer100ms the velocity to set in ticks per 100ms.
     */
    public void setVelocity(double ticksPer100ms) {
        logger.stateChange("Velocity Set", ticksPer100ms);
        tab.setEntry("setVelTicks", ticksPer100ms);
        multiPID.selectConfig(MultiPID.Type.VELOCITY);
        turretTalon.set(ControlMode.Velocity, ticksPer100ms);
        tab.setEntry("ctrlTarget", turretTalon.getClosedLoopTarget());
    }

    public MotorTemp getTemp() {
        return new MotorTemp(turretTalon.getDeviceID(), turretTalon.getTemperature(), "turretTalon");
    }

    @Override
    public void setPower(double pwr) {
        logger.powerChange(pwr);
        turretTalon.set(ControlMode.PercentOutput, pwr);
    }

    /**
     * Checks for turret overshoot / undershoot encoder issue.
     * Encoder will jump +/- 4096 ticks (full rotation) during runtime.
     * Fixes by offsetting returned values and modifying soft limits.
     */
    private void checkWrapError() {
        final int lastWrapOffset = wrapErrOffset;
        final double currPos = turretTalon.getSelectedSensorPosition();
        boolean isOverRotated = currPos > FORWARD_LIMIT_TICKS + DETECT_BUFFER_TICKS;
        boolean isUnderRotated = currPos < BACK_LIMIT_TICKS - DETECT_BUFFER_TICKS;
        this.wrapErrOffset = isOverRotated ? -4096 : isUnderRotated ? 4096 : 0;

        if (lastWrapOffset != wrapErrOffset) {
            tab.setEntry("Wrap Error", isOverRotated ? "Over" : isUnderRotated ? "Under" : "None");
            MotorUtil.setSoftLimits(FORWARD_LIMIT_TICKS - wrapErrOffset,
                    BACK_LIMIT_TICKS - wrapErrOffset, turretTalon);
        }
    }

    @Override
    public void periodic() {
        // Runs every 5s after init (5s/0.02s=250x)
        errLoopCtr++;
        if (errLoopCtr >= 250) {
            checkWrapError();
            errLoopCtr = 0;
        }
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}