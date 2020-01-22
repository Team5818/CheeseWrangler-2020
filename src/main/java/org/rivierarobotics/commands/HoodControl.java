package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.util.MathUtil;

public class HoodControl extends CommandBase
{
	private final Hood hood;
	private final Joystick leftCoDriverJs;

	public HoodControl(Hood hood)
	{
		this.hood = hood;
		this.leftCoDriverJs = Robot.runningRobot.coDriverLeftJs;
		addRequirements(hood);
		//TODO make each control command the default for Hood and Turret, and remove the control button
	}

	@Override
	public void execute()
	{
		hood.setManualPower(MathUtil.fitDeadband(leftCoDriverJs.getY()));
	}

	@Override
	public boolean isFinished()
	{
		return false;
	}
}
