package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import jaci.pathfinder.followers.EncoderFollower;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;

public class SideBasedPathfinder extends CommandBase {
    public EncoderFollower left;
    protected EncoderFollower right;
    private double wheel_diameter = 4.0;

    public SideBasedPathfinder(@Provided EncoderFollower left, @Provided EncoderFollower right, @Provided DriveTrain dt){
        this.left = left;
        this.right = right;
        //left.configureEncoder();

    }




}

