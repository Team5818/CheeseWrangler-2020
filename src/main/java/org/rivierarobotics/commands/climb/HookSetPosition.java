package org.rivierarobotics.commands.climb;

import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.MotionMagicSetPosition;
import org.rivierarobotics.subsystems.Climb;
import org.rivierarobotics.subsystems.Hook;
import org.rivierarobotics.util.RobotShuffleboard;

@GenerateCreator
public class HookSetPosition extends MotionMagicSetPosition<Hook> {
    public HookSetPosition(@Provided Hook hook, @Provided RobotShuffleboard shuffleboard, double position) {
        super(hook, hook::getPositionTicks, hook::setPositionTicks, position, 50, 5, shuffleboard);
    }
}

