package org.rivierarobotics.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.CheeseWheelCommands;
import org.rivierarobotics.commands.DriveCommands;

import javax.inject.Inject;

public class ForwardAuto extends SequentialCommandGroup {

    @Inject
    public ForwardAuto(DriveCommands commands, CheeseWheelCommands cheeseWheelCommands) {
        addCommands(
                commands.driveDistance(1).withTimeout(1.0)
        );
    }
}
