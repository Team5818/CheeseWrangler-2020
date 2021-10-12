package org.rivierarobotics.autonomous.advanced;

import edu.wpi.first.wpilibj2.command.*;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.VisionTarget;

public class CenterShoot extends CommandBase {
    private Command autoCommand;
    private final DriveCommands driveCommands;
    private final VisionCommands visionCommands;
    private final CheeseWheelCommands cheeseWheelCommands;
    private final CollectionCommands collectionCommands;

    public CenterShoot(@Provided DriveCommands driveCommands,
                       @Provided VisionCommands visionCommands,
                       @Provided CheeseWheelCommands cheeseWheelCommands,
                       @Provided CollectionCommands collectionCommands,
                       @Provided NavXGyro gyro) {
        this.visionCommands = visionCommands;
        this.driveCommands = driveCommands;
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.collectionCommands = collectionCommands;
        //gyro.setAngleAdjustment(180);
    }

    @Override
    public void initialize() {
        this.autoCommand = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        visionCommands.calcAim(VisionTarget.TOP),
                        new SequentialCommandGroup(
                                driveCommands.driveDistance(1.42, 0.45),
                                cheeseWheelCommands.shootUntilEmpty(),
                                driveCommands.driveDistance(1.38, 0.45),
                                driveCommands.rotateTo(180-34.7),

                                new ParallelDeadlineGroup(
                                        driveCommands.driveDistance(1.4, 0.2),
                                        collectionCommands.continuous(CheeseWheel.AngleOffset.COLLECT_BACK)),
                                driveCommands.rotateTo(180+56.7),

                                driveCommands.driveDistance(-1.69, 0.45),

                                cheeseWheelCommands.shootUntilEmpty()
                        )));

        autoCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return !CommandScheduler.getInstance().isScheduled(autoCommand);
    }
}

