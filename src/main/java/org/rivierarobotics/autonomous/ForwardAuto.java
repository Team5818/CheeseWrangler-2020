package org.rivierarobotics.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.CheeseWheelCommands;
import org.rivierarobotics.commands.DriveCommands;
import org.rivierarobotics.commands.IntakeCommands;
import org.rivierarobotics.util.Side;
import org.rivierarobotics.util.VisionTarget;

import javax.inject.Inject;

public class ForwardAuto extends SequentialCommandGroup {

    @Inject
    public ForwardAuto(DriveCommands commands,
                       CheeseWheelCommands cheeseWheelCommands,
                       IntakeCommands intake) {
        addCommands(
            cheeseWheelCommands.shootNWedges(VisionTarget.INNER, 5),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    commands.driveDistance(-3).withTimeout(4.0),
                    commands.driveDistance(3).withTimeout(4.0)
                ),
                intake.setPower(Side.BACK)
            ),
            cheeseWheelCommands.shootNWedges(VisionTarget.INNER, 5)
        );
    }
}
