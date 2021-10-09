package org.rivierarobotics.autonomous.advanced;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class SixBallTrench extends SequentialCommandGroup {
    public SixBallTrench(@Provided DriveCommands driveCommands,
                         @Provided VisionCommands visionCommands,
                         @Provided CheeseWheelCommands cheeseWheelCommands,
                         @Provided CollectionCommands collectionCommands) {
        super(
                visionCommands.correctPosition(),
                new ParallelCommandGroup(
                        visionCommands.calcAim(VisionTarget.TOP),
                        new SequentialCommandGroup(
                                cheeseWheelCommands.shootUntilEmpty(),
                                driveCommands.rotateTo(-45),
                                driveCommands.driveDistance(-1.0, 0.3),
                                driveCommands.rotateTo(0),
                                new ParallelDeadlineGroup(
                                        driveCommands.driveDistance(-1.5, 0.2),
                                        collectionCommands.continuous(CheeseWheel.AngleOffset.COLLECT_BACK)),
                                driveCommands.driveDistance(1, 0.3),
                                cheeseWheelCommands.shootUntilEmpty()
                                )));
    }
}
