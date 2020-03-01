package org.rivierarobotics.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.CheeseWheelCommands;
import org.rivierarobotics.commands.DriveCommands;
import org.rivierarobotics.commands.IntakeCommands;
import org.rivierarobotics.commands.VisionCommands;
import org.rivierarobotics.util.Side;
import org.rivierarobotics.util.VisionTarget;

import javax.inject.Inject;

public class ForwardAuto extends SequentialCommandGroup {

    @Inject
    public ForwardAuto(DriveCommands commands,
                       CheeseWheelCommands cheeseWheelCommands,
                       IntakeCommands intake,
                       VisionCommands vision) {
        addCommands(
            cheeseWheelCommands.shootNWedges(VisionTarget.INNER, 5),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    commands.driveDistance(-4).withTimeout(6.0),
                    commands.driveDistance(4).withTimeout(6.0)
                ),
                intake.setPower(Side.BACK)
            ),
            vision.visionAim(VisionTarget.INNER).withTimeout(1.0),
            cheeseWheelCommands.shootNWedges(VisionTarget.INNER, 5)
        );
    }
}
