package org.rivierarobotics.autonomous.advanced;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.commands.flywheel.FlywheelCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class TriangleAdvanced extends ParallelDeadlineGroup {
    public TriangleAdvanced(@Provided DriveCommands driveCommands, @Provided FlywheelCommands flywheelCommands,
                            @Provided CheeseWheelCommands cheeseWheelCommands, @Provided VisionCommands visionCommands,
                            @Provided CollectionCommands collectionCommands) {
        super(
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                driveCommands.driveDistance(-0.75, -0.3),
                                driveCommands.driveDistance(-0.75, -0.1),
                                driveCommands.rotateTo(90),
                                driveCommands.driveDistance(1, 0.5))),
                                collectionCommands.continuous(CheeseWheel.AngleOffset.COLLECT_BACK)),

                        new SequentialCommandGroup(
                                visionCommands.correctPosition(),
                                visionCommands.calcAim(VisionTarget.TOP),
                                cheeseWheelCommands.shootUntilEmpty()));
    }
}