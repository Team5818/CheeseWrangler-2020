package org.rivierarobotics.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.*;
import org.rivierarobotics.util.Side;
import org.rivierarobotics.util.VisionTarget;

import javax.inject.Inject;


public class TrenchRun extends SequentialCommandGroup {

    @Inject
    public TrenchRun(AutonomousCommands autonomousCommands,
                     VisionCommands visionCommands,
                     CWCommandGroups cheesewheelCommandGroups,
                     IntakeCommands intakeCommands,
                     LimelightServoCommands limelightServoCommands,
                     TurretCommands turretCommands) {
        addCommands(
                limelightServoCommands.setAngle(30),
                turretCommands.setAngle(180),
                visionCommands.correctPosition(),
                autonomousCommands.pathweaver(Pose2dPath.MOVETOSHOOT),
                visionCommands.visionAim(VisionTarget.INNER),
                cheesewheelCommandGroups.shootNext(),
                cheesewheelCommandGroups.shootNext(),
                cheesewheelCommandGroups.shootNext(),

                intakeCommands.setPower(Side.BACK),
                autonomousCommands.pathweaver(Pose2dPath.MOVETOCOLLECT),
                cheesewheelCommandGroups.autoCollect(false),

                autonomousCommands.pathweaver(Pose2dPath.NEXTBAL),
                cheesewheelCommandGroups.autoCollect(false),

                autonomousCommands.pathweaver(Pose2dPath.NEXTBALL),
                cheesewheelCommandGroups.autoCollect(false),

                autonomousCommands.pathweaver(Pose2dPath.SHOOTCOLLECTED),
                visionCommands.visionAim(VisionTarget.INNER),
                cheesewheelCommandGroups.shootNext(),
                cheesewheelCommandGroups.shootNext(),
                cheesewheelCommandGroups.shootNext(),

                autonomousCommands.pathweaver(Pose2dPath.NEXTBALLL),
                intakeCommands.setPower(Side.FRONT),
                cheesewheelCommandGroups.autoCollect(true),

                autonomousCommands.pathweaver(Pose2dPath.NEXTBALLLL),
                intakeCommands.setPower(Side.FRONT),
                cheesewheelCommandGroups.autoCollect(true),

                autonomousCommands.pathweaver(Pose2dPath.SHOOTCOLLECTEDD),
                cheesewheelCommandGroups.shootNext(),
                cheesewheelCommandGroups.shootNext());
    }
}
