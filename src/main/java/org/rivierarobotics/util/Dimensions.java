/*
 * This file is part of Placeholder-2020, licensed under the GNU General Public License (GPLv3).
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

package org.rivierarobotics.commands;

<<<<<<< HEAD:src/main/java/org/rivierarobotics/commands/EjectorEjectCheese.java
import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.Ejector;

@GenerateCreator
public class EjectorEjectCheese extends InstantCommand {
    private final Ejector ejector;

    public EjectorEjectCheese(@Provided Ejector ejector) {
        this.ejector = ejector;
        addRequirements(ejector);
    }

    @Override
    public void execute() {
        ejector.setPower(0.75);
    }
}
=======
public interface Dimensions {
    double WHEEL_CIRCUMFERENCE = 0.32; // meters
    double TRACKWIDTH = 0.7366; // meters
}
>>>>>>> master:src/main/java/org/rivierarobotics/util/Dimensions.java
