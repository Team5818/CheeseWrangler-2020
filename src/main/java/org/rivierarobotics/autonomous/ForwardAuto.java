package org.rivierarobotics.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.CheeseWheelCommands;
import org.rivierarobotics.commands.DriveCommands;
import org.rivierarobotics.util.VisionTarget;

import javax.inject.Inject;

public class ForwardAuto extends SequentialCommandGroup {

    @Inject
    public ForwardAuto(DriveCommands commands, CheeseWheelCommands cheeseWheelCommands) {
        addCommands(
            cheeseWheelCommands.shootNWedges(VisionTarget.INNER, 5),
            commands.driveDistance(-1).withTimeout(1.0)
        );
    }
}
