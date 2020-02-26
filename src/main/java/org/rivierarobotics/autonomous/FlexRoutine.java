package org.rivierarobotics.autonomous;

import javax.inject.Inject;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.CheeseWheelCommands;
import org.rivierarobotics.commands.VisionCommands;
import org.rivierarobotics.util.VisionTarget;



public class FlexRoutine extends SequentialCommandGroup {

    @Inject
    public FlexRoutine(AutonomousCommands autonomousCommands,
                       VisionCommands visionCommands,
                       CheeseWheelCommands cheeseWheelCommands) {
        addCommands(
            autonomousCommands.pathweaver(Pose2dPath.FLEX),
            visionCommands.visionAim(VisionTarget.INNER),
            cheeseWheelCommands.autoCollect(true),
            cheeseWheelCommands.shootNext()
        );
    }
}
