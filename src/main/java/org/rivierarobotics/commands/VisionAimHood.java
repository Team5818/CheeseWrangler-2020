package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.util.VisionUtil;

public class VisionAimHood extends CommandBase {
    public Hood hood;
    public VisionAimHood(Hood hd){
        this.hood = hd;
        addRequirements(hood);
    }
    @Override
    public void execute(){
        double ty = VisionUtil.getLLValue("ty");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
