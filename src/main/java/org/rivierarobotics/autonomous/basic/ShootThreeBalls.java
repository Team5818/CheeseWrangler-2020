package org.rivierarobotics.autonomous.basic;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.flywheel.FlywheelCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class ShootThreeBalls extends SequentialCommandGroup {
    public ShootThreeBalls(@Provided VisionCommands visionCommands,
                         @Provided CheeseWheelCommands cheeseWheel, @Provided FlywheelCommands flywheel) {
        super(
                new ParallelDeadlineGroup(
                        flywheel.setVelocity(),
                        sequence(new WaitCommand(1),
                                visionCommands.correctPosition(),
                                visionCommands.calcAim(VisionTarget.INNER),
                                cheeseWheel.shootNWedges(3)
                        )
                )
        );
    }
}

