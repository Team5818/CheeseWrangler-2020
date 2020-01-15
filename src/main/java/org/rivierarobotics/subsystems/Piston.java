package org.rivierarobotics.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import org.rivierarobotics.util.RobotMap;

public class Piston
{
	private Solenoid sol;

	public Piston()
	{
		sol = new Solenoid(RobotMap.CODRIVER_LEFT_JS);
	}

	public void extend(Boolean extend)
	{
		sol.set(extend);
	}
}
