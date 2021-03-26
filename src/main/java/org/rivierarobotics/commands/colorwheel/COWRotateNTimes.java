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

package org.rivierarobotics.commands.colorwheel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.ColorWheel;

@GenerateCreator
public class COWRotateNTimes extends CommandBase {
    protected static final int NUM_COLOR_SLICES_PER_ROT = 8;
    protected static final int TIMEOUT_SECONDS = 20;
    protected final ColorWheel colorWheel;
    protected final double rotations;
    private final double rotatePower;
    protected int colorChangeCtr;
    protected ColorWheel.GameColor lastColor;
    protected double startTime;

    public COWRotateNTimes(@Provided ColorWheel colorWheel, double rotations, double rotatePower) {
        this.colorWheel = colorWheel;
        this.rotations = rotations;
        this.rotatePower = rotatePower;
        addRequirements(colorWheel);
    }

    public COWRotateNTimes(@Provided ColorWheel colorWheel, double rotations) {
        this(colorWheel, rotations, 1.0);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        colorWheel.setPower(rotatePower);
        ColorWheel.GameColor currentColor = colorWheel.getGameColor();
        if (currentColor != ColorWheel.GameColor.NULL && (lastColor == null || !lastColor.equals(currentColor))) {
            colorChangeCtr++;
            lastColor = currentColor;
        }
        SmartDashboard.putNumber("cchange", colorChangeCtr);
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return colorChangeCtr > rotations * NUM_COLOR_SLICES_PER_ROT
                || Timer.getFPGATimestamp() - startTime > TIMEOUT_SECONDS * rotations;
    }
}
