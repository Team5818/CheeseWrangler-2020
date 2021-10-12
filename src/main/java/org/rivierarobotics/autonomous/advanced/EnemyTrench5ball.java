package org.rivierarobotics.autonomous.advanced;

import edu.wpi.first.wpilibj2.command.*;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Climb;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.PositionTracker;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class EnemyTrench5ball extends CommandBase {
    private Command autoCommand;
    private final DriveCommands driveCommands;
    private final VisionCommands visionCommands;
    private final CheeseWheelCommands cheeseWheelCommands;
    private final CollectionCommands collectionCommands;
    private final NavXGyro gyro;

    public EnemyTrench5ball(@Provided DriveCommands driveCommands,
                            @Provided VisionCommands visionCommands,
                            @Provided CheeseWheelCommands cheeseWheelCommands,
                            @Provided CollectionCommands collectionCommands,
                            @Provided NavXGyro gyro) {
        this.visionCommands = visionCommands;
        this.driveCommands = driveCommands;
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.collectionCommands = collectionCommands;
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        gyro.setAngleAdjustment(180);
        PositionTracker.setPosition(new double[]{3.62, 2.28});
//        this.autoCommand = new SequentialCommandGroup(
//                new ParallelDeadlineGroup(
//                        visionCommands.calcAim(VisionTarget.TOP),
//                        new SequentialCommandGroup(
//                                driveCommands.rotateTo(180+39.8),
//                                driveCommands.driveDistance(-1.45, 0.3),
//                                cheeseWheelCommands.shootUntilEmpty(),
//                                driveCommands.driveDistance(-1.48, 0.3),
//                                driveCommands.rotateTo(180),
//                                new ParallelDeadlineGroup(
//                                        driveCommands.driveDistance(-0.85, 0.2),
//                                        collectionCommands.continuous(CheeseWheel.AngleOffset.COLLECT_BACK)),
//                                cheeseWheelCommands.shootUntilEmpty()
//                        )));

        this.autoCommand = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        visionCommands.calcAim(VisionTarget.TOP),
                        new SequentialCommandGroup(
                                driveCommands.driveDistance(0.8, 0.35),
                                driveCommands.driveDistance(0.8, 0.6),
                                driveCommands.driveDistance(0.8, 0.35),
                                new ParallelDeadlineGroup(
                                        new SequentialCommandGroup(
                                                driveCommands.driveDistance(1.2, 0.15),
                                                new WaitCommand(1.5)
                                        ),
                                        collectionCommands.continuous(CheeseWheel.AngleOffset.COLLECT_FRONT)),
                                driveCommands.rotateTo(220),
                                driveCommands.driveDistance(-1.6, 0.4),
                                driveCommands.rotateTo(180),
                                cheeseWheelCommands.shootUntilEmpty()))
                        );

//        this.autoCommand = new SequentialCommandGroup(
//                new ParallelDeadlineGroup(
//                        visionCommands.calcAim(VisionTarget.TOP),
//                        new SequentialCommandGroup(
//                                driveCommands.rotateTo(90),
//                                new WaitCommand(2),
//                                driveCommands.rotateTo(180),
//                                new WaitCommand(2),
//                                driveCommands.rotateTo(270),
//                                new WaitCommand(2),
//                                driveCommands.rotateTo(360),
//                                new WaitCommand(2),
//                                driveCommands.rotateTo(20)
//                                )));


        CommandScheduler.getInstance().schedule(autoCommand);
    }

    @Override
    public boolean isFinished() {
        return !CommandScheduler.getInstance().isScheduled(autoCommand);
    }
}
