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

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Intake;
import org.rivierarobotics.util.BallTracker;

@GenerateCreator
public class CollectInfiniteWedges extends CommandBase {
    private final Intake intake;
    private final CheeseWheel cheeseWheel;
    private final CheeseWheelCommands cheeseWheelCommands;
    private final double frontPower;
    private final double backPower;
    private static final double pwrConstant = 1.0;
    private final CheeseWheel.AngleOffset mode;
    private final BallTracker ballTracker;
    private final int direction;

    CollectInfiniteWedges(@Provided Intake intake, @Provided CheeseWheel cheeseWheel,
                          @Provided CheeseWheelCommands cheeseWheelCommands,
                          @Provided BallTracker ballTracker, CheeseWheel.AngleOffset mode) {
        this.intake = intake;
        this.cheeseWheel = cheeseWheel;
        this.ballTracker = ballTracker;
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.mode = mode;

        switch (mode) {
            case COLLECT_FRONT:
                frontPower = pwrConstant;
                backPower = 0.0;
                direction = -1;
                break;
            case COLLECT_BACK:
                frontPower = 0.0;
                backPower = pwrConstant;
                direction = 1;
                break;
            default:
                throw new IllegalArgumentException("Invalid side");
        }

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (!ballTracker.frontOnIndex) {
            moveToNext();
        }
    }

    @Override
    public void execute() {
        intake.setPower(frontPower, backPower);

        if (!cheeseWheel.getPidController().atSetpoint()
            || (mode == CheeseWheel.AngleOffset.COLLECT_BACK && !ballTracker.backOnIndex)
            || (mode == CheeseWheel.AngleOffset.COLLECT_FRONT && !ballTracker.frontOnIndex)) {
            return;
        }

        if (cheeseWheel.isFrontBallPresent()) {
            moveToNext();
        }
    }

    private void moveToNext() {
        cheeseWheelCommands.moveToNextIndex(direction, mode).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0, 0);
    }
}
