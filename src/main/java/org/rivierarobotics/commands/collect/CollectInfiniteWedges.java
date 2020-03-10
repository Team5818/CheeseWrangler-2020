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

package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Intake;
import org.rivierarobotics.util.BallTracker;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.Side;

import java.util.function.BooleanSupplier;

@GenerateCreator
public class CollectInfiniteWedges extends CommandBase {
    private final Intake intake;
    private final CheeseWheel cheeseWheel;
    private final CheeseWheelCommands cheeseWheelCommands;
    private final double frontPower;
    private final double backPower;
    private static final double pwrConstant = 1.0;
    private final BooleanSupplier hasBall;
    private final CheeseWheel.AngleOffset mode;
    private final BallTracker ballTracker;
    private final int direction;
    private double startSeen;

    CollectInfiniteWedges(@Provided Intake intake, @Provided CheeseWheel cheeseWheel,
                          @Provided CheeseWheelCommands cheeseWheelCommands, @Provided BallTracker ballTracker, CheeseWheel.AngleOffset mode) {
        this.intake = intake;
        this.cheeseWheel = cheeseWheel;
        this.ballTracker = ballTracker;
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.mode = mode;
        if (mode.equals(CheeseWheel.AngleOffset.COLLECT_FRONT)) {
            frontPower = pwrConstant;
            backPower = 0.0;
            hasBall = cheeseWheel::isFrontBallPresent;
            direction = -1;
        } else {
            if (mode != CheeseWheel.AngleOffset.COLLECT_BACK) {
                throw new IllegalArgumentException("Invalid side");
            }
            frontPower = 0.0;
            backPower = pwrConstant;
            hasBall = cheeseWheel::isBackBallPresent;
            direction = 1;
        }
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        int currentSlot = cheeseWheel.getIndex(mode);
        if(!ballTracker.frontOnIndex) {
            moveToNext();
        }
    }

    @Override
    public void execute() {
        intake.setPower(frontPower, backPower);
        if (!cheeseWheel.getPidController().atSetpoint() && !ballTracker.frontOnIndex) {
            return;
        }
        if (cheeseWheel.isFrontBallPresent()) {
            ballTracker.addBall(cheeseWheel.getIndex(mode));
            moveToNext();
        }
    }

    private void moveToNext() {
        cheeseWheelCommands.moveToNextIndex(direction,mode).schedule();
        //cheeseWheel.addAngle(cheeseWheel.getAngleAdded(cheeseWheel.getIndex(mode) + direction,mode));
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0, 0);
    }
}
