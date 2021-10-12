package org.rivierarobotics.autonomous.advanced;

import edu.wpi.first.wpilibj2.command.*;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.PositionTracker;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class OffsetSixBallTrench extends CommandBase {
    private Command autoCommand;
    private final DriveCommands driveCommands;
    private final VisionCommands visionCommands;
    private final CheeseWheelCommands cheeseWheelCommands;
    private final CollectionCommands collectionCommands;

    public OffsetSixBallTrench(@Provided DriveCommands driveCommands,
                               @Provided VisionCommands visionCommands,
                               @Provided CheeseWheelCommands cheeseWheelCommands,
                               @Provided CollectionCommands collectionCommands) {
        this.visionCommands = visionCommands;
        this.driveCommands = driveCommands;
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.collectionCommands = collectionCommands;
    }

    @Override
    public void initialize() {
        PositionTracker.setPosition(new double[]{2, 2});
        this.autoCommand = new ParallelDeadlineGroup(
                visionCommands.calcAim(VisionTarget.TOP),
                new SequentialCommandGroup(
                        new WaitCommand(0.1),
                        cheeseWheelCommands.shootUntilEmpty(),
                        driveCommands.driveDistance(-1.0, 0.3),
                        new ParallelDeadlineGroup(
                                driveCommands.driveDistance(-1.5, 0.2),
                                collectionCommands.continuous(CheeseWheel.AngleOffset.COLLECT_BACK)),
                        driveCommands.driveDistance(1, 0.3),
                        cheeseWheelCommands.shootUntilEmpty()
                ));

        autoCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return !CommandScheduler.getInstance().isScheduled(autoCommand);
    }
}
