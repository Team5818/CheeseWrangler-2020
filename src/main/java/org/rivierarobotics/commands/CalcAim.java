package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.PositionTracker;
import org.rivierarobotics.util.ShooterUtil;
import org.rivierarobotics.util.VisionUtil;

@GenerateCreator
public class CalcAim extends CommandBase {
    private final Hood hood;
    private final DriveTrain driveTrain;
    private final Flywheel flywheel;
    private final VisionUtil vision;
    private final Turret turret;
    private final PositionTracker tracker;
    private final double extraDistance;

    public CalcAim(@Provided Hood hood, @Provided DriveTrain dt, @Provided Flywheel flywheel,
                   @Provided VisionUtil vision, @Provided Turret turret, @Provided PositionTracker tracker,
                   double extraDistance) {
        this.hood = hood;
        this.driveTrain = dt;
        this.flywheel = flywheel;
        this.vision = vision;
        this.turret = turret;
        this.tracker = tracker;
        this.extraDistance = extraDistance;
        addRequirements(hood, flywheel, turret);
    }

    @Override
    public void execute() {
        double y = ShooterUtil.getYVelocityConstant();
        double [] pos = tracker.getPosition();
        double xFromGoal = ShooterUtil.getFieldLength() - pos[1];
        double zFromGoal = ShooterUtil.getLeftFieldToGoal() - pos[0];
        double dist = Math.sqrt(Math.pow(xFromGoal, 2) + Math.pow(zFromGoal, 2));
        SmartDashboard.putNumber("dist", dist);
        double t = ShooterUtil.getTConstant();
        double vx = (extraDistance + xFromGoal) / t - driveTrain.getYVelocity();
        double vz = zFromGoal / t - driveTrain.getXVelocity();
        double vxz = Math.sqrt(Math.pow(vx, 2) + Math.pow(vz, 2));
        double hoodAngle = Math.toDegrees(Math.atan2(y, vxz));
        double turretAngle = Math.toDegrees(Math.atan2(vz, vx));
        double ballVel = vxz / Math.cos(Math.toRadians(hoodAngle));
        double encoderVelocity = ShooterUtil.velocityToTicks(ballVel);
        double captainKalbag = captainKalbag(xFromGoal,zFromGoal);

        SmartDashboard.putNumber("changeInAngle", captainKalbag);
        SmartDashboard.putNumber("changeInAngleDegrees", Math.toDegrees(captainKalbag));
        SmartDashboard.putNumber("BallVel", ballVel);
        SmartDashboard.putNumber("FlyVel", encoderVelocity + 10);
        SmartDashboard.putNumber("HoodAngleMath", hoodAngle);

        if(driveTrain.getAvgVelocity() > 60) {
            turret.changeAimMode(Turret.AimMode.MOVING);
            turret.setPositionTicks(captainKalbag * turret.getAnglesOrInchesToTicks() / 10);
        } else {
            turret.changeAimMode(Turret.AimMode.STILL);
            turret.setAbsolutePosition(turretAngle);
        }
        if (hoodAngle <= ShooterUtil.getMaxHoodAngle() && encoderVelocity <= ShooterUtil.getMaxFlywheelVelocity()) {
            hood.setAbsolutePosition(hoodAngle + 3.5);
            flywheel.setPositionTicks(encoderVelocity + 10);
        } else {
            if (dist < 1) {
                hood.setAbsolutePosition(ShooterUtil.getMaxHoodAngle());
                flywheel.setPositionTicks(120);
            } else {
                if (dist > 1) {
                    hood.setAbsolutePosition(hoodAngle);
                    flywheel.setPositionTicks(ShooterUtil.getMaxFlywheelVelocity());
                }
            }
        }
    }

    public double captainKalbag(double xFromGoal, double zFromGoal) {
        double epicTime = 0.1;
        double xDist = xFromGoal - driveTrain.getYVelocity() * epicTime;
        double zDist = zFromGoal - driveTrain.getXVelocity() * epicTime;
        return ( 1/(Math.pow((zDist / xDist),2) + 1) ) * ( (-driveTrain.getXVelocity() * xDist)
                - (-driveTrain.getYVelocity() * xDist) ) / Math.pow(xDist, 2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
