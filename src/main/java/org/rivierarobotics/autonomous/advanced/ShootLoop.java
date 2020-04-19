package org.rivierarobotics.autonomous.advanced;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.autonomous.AutonomousCommands;
import org.rivierarobotics.autonomous.Pose2dPath;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class ShootLoop extends SequentialCommandGroup {
    public ShootLoop(@Provided AutonomousCommands auto, @Provided CollectionCommands collect,
                     @Provided CheeseWheelCommands shoot, @Provided VisionCommands aim, Pose2dPath loop) {
        super(
                auto.pathweaver(loop)
                        .deadlineWith(collect.continuous(CheeseWheel.AngleOffset.COLLECT_FRONT))
                        .andThen(aim.visionAim(VisionTarget.INNER))
                        .andThen(shoot.shootNWedges(5))
        );
    }
}
