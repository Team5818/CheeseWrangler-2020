package org.rivierarobotics.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.CWCommandGroups;
import org.rivierarobotics.commands.LimelightServoCommands;
import org.rivierarobotics.commands.TurretCommands;
import org.rivierarobotics.commands.VisionCommands;
import org.rivierarobotics.util.NavXGyro;
import org.rivierarobotics.util.VisionTarget;

import javax.inject.Inject;

public class BasicAuto extends SequentialCommandGroup {
    @Inject
    public BasicAuto(AutonomousCommands autonomousCommands,
                     VisionCommands visionCommands,
                     CWCommandGroups cheesewheelCommandGroups,
                     LimelightServoCommands limelightServoCommands,
                     TurretCommands turretCommands){
        addCommands(
                limelightServoCommands.setAngle(30),
                turretCommands.setAngle(180),
                visionCommands.correctPosition(),
                autonomousCommands.pathweaver(Pose2dPath.MOVETOSHOOTT),
                visionCommands.visionAim(VisionTarget.INNER),
                cheesewheelCommandGroups.shootNext(),
                cheesewheelCommandGroups.shootNext(),
                cheesewheelCommandGroups.shootNext()
        );
    }
}
