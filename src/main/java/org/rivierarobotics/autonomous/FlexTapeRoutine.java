package org.rivierarobotics.autonomous;


import javax.inject.Inject;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.CheeseWheelCommands;
import org.rivierarobotics.commands.VisionCommands;
import org.rivierarobotics.util.VisionTarget;

public class FlexTapeRoutine extends SequentialCommandGroup {

    @Inject

    public FlexTapeRoutine(AutonomousCommands autonomousCommands,
                           VisionCommands visionCommands,
                           CheeseWheelCommands cheeseWheelCommands){

        addCommands(
                autonomousCommands.pathweaver(Pose2dPath.FLEXTAPE),
                visionCommands.visionAim(VisionTarget.INNER),
                cheeseWheelCommands.autoCollect(true),
                cheeseWheelCommands.shootNext()
        );
    }
}
