package org.rivierarobotics.commands.cheesewheel;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.ShooterUtil;
import org.rivierarobotics.util.VisionTarget;

import javax.inject.Inject;

@GenerateCreator
public class CheeseWheelCommandCanceler extends InstantCommand {

    private final CheeseWheelCommands commands;
    private final int direction;
    private final CheeseWheel.AngleOffset mode;

    public CheeseWheelCommandCanceler(@Provided CheeseWheelCommands commands, int direction, CheeseWheel.AngleOffset mode) {
        this.commands = commands;
        this.direction = direction;
        this.mode = mode;
    }

    @Override
    public void initialize() {
        commands.moveToNextIndex(direction,mode).schedule();
    }
}