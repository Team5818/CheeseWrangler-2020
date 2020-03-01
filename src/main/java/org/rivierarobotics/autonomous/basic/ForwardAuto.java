package org.rivierarobotics.autonomous.basic;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.CheeseWheelCommands;
import org.rivierarobotics.commands.DriveCommands;
import org.rivierarobotics.commands.IntakeCommands;
import org.rivierarobotics.commands.VisionCommands;
import org.rivierarobotics.util.Side;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class ForwardAuto extends SequentialCommandGroup {

    public ForwardAuto(@Provided DriveCommands commands,
                       @Provided CheeseWheelCommands cheeseWheelCommands,
                       @Provided IntakeCommands intake,
                       @Provided VisionCommands vision,
                       boolean useVisionAim) {
        addCommands(
            cheeseWheelCommands.shootNWedges(VisionTarget.INNER, 5),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    commands.driveDistance(-5, 0.25).withTimeout(6.0),
                    commands.driveDistance(5, 0.5).withTimeout(6.0)
                ),
                intake.setPower(Side.BACK)
            ),
            useVisionAim
                ? vision.visionAim(VisionTarget.INNER).withTimeout(1.0)
                : new InstantCommand(),
            cheeseWheelCommands.shootNWedges(VisionTarget.INNER, 5)
        );
    }
}
