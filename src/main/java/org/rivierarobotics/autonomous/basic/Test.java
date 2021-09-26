package org.rivierarobotics.autonomous.basic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.autonomous.AutonomousCommands;
import org.rivierarobotics.autonomous.Pose2dPath;

@GenerateCreator
public class Test extends SequentialCommandGroup {

    public Test(@Provided AutonomousCommands auto){
        super(
                auto.pathtracer(Pose2dPath.STRAIGHT)
        );
    }
}
