package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.ColorWheel;

import javax.inject.Inject;

public class ColorWheelControl extends CommandBase {
    private final ColorWheel colorWheel;

    @Inject
    public ColorWheelControl(ColorWheel colorWheel){
        this.colorWheel = colorWheel;
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){ return false; }
}
