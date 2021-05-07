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

package org.rivierarobotics.autonomous;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;
import org.rivierarobotics.util.Vec2D;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.util.LinkedList;
import java.util.List;

/**
 * Records a path traveled based on velocity of each wheel side. Writes to a
 * file on the RoboRio at ~/paths/ titled GEN_[mstime]. Records tangent
 * velocity to be used for Quintic Hermite spline trajectory generation. Use
 * with PathTracer (a compatible JSON will be the output).
 */
@GenerateCreator
public class RecordPath extends CommandBase {
    private final DriveTrain driveTrain;
    private final Joystick driverLeft;
    private final Joystick driverRight;
    private final Joystick driverButtons;
    private final RobotShuffleboardTab tab;
    private final boolean isPower;
    private List<Vec2D> vel;
    private String recName;

    public RecordPath(@Provided DriveTrain driveTrain,
                      @Provided @Input(Input.Selector.DRIVER_LEFT) Joystick driverLeft,
                      @Provided @Input(Input.Selector.DRIVER_RIGHT) Joystick driverRight,
                      @Provided @Input(Input.Selector.DRIVER_BUTTONS) Joystick driverButtons,
                      @Provided RobotShuffleboard shuffleboard, boolean isPower) {
        this.driveTrain = driveTrain;
        this.driverLeft = driverLeft;
        this.driverRight = driverRight;
        this.driverButtons = driverButtons;
        this.isPower = isPower;
        this.tab = shuffleboard.getTab("PathTracer");
    }

    @Override
    public void initialize() {
        vel = new LinkedList<>();
        recName = "GEN_" + System.currentTimeMillis();
        tab.setEntry("Rec Name", recName);
        tab.setEntry("Rec Path", true);
    }

    @Override
    public void execute() {
        if (isPower) {
            vel.add(new Vec2D(driverRight.getX(), driverLeft.getY()));
        } else {
            vel.add(new Vec2D(driveTrain.getXVelocity(), driveTrain.getYVelocity()));
        }
    }

    private static Path getPathDir() {
        return RobotBase.isReal()
                ? Filesystem.getDeployDirectory().toPath().resolve("paths/")
                : Filesystem.getLaunchDirectory().toPath().resolve("PathWeaver/Paths");
    }

    private static void writeCsvString(BufferedWriter file, Object... data) throws IOException {
        for (Object d : data) {
            file.write(d.toString() + ",");
        }
        file.write("\n");
        file.flush();
    }

    @Override
    public void end(boolean interrupted) {
        File out = getPathDir().resolve(recName).toFile();
        tab.setEntry("outDir", out.getAbsolutePath());
        try (BufferedWriter file = new BufferedWriter(new FileWriter(out))) {
            if (isPower) {
                for (Vec2D vPair : vel) {
                    writeCsvString(file, vPair.getX(), vPair.getY());
                }
            } else {
                file.write("X,Y,Tangent X,Tangent Y,Fixed Theta,Reversed,Name\n");
                double vxAccum = 0;
                double vyAccum = 0;
                for (Vec2D vPair : vel) {
                    vxAccum += vPair.getX() * SplinePath.RIO_LOOP_TIME_MS;
                    vxAccum += vPair.getY() * SplinePath.RIO_LOOP_TIME_MS;
                    writeCsvString(file, vxAccum, vyAccum, vPair.getX(), vPair.getY(), true, false);
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        tab.setEntry("Rec Path", false);
    }

    @Override
    public boolean isFinished() {
        return driverButtons.getRawButton(5);
    }
}
