package org.rivierarobotics.commands.shooting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.subsystems.CheeseWheel;

@GenerateCreator
public class ShootUntilEmpty extends CommandBase{
        private Command command;
        private final CheeseWheel wheel;
        private CheeseWheelCommands commands;

        public ShootUntilEmpty(@Provided CheeseWheel cheese, @Provided CheeseWheelCommands comm) {
            this.wheel = cheese;
            this.commands= comm;
        }

        @Override
        public void execute() {
            if(wheel.hasBall() && (command == null || !CommandScheduler.getInstance().isScheduled(command))) {
                Command c = commands.continuousShoot();
                CommandScheduler.getInstance().schedule(c);
                this.command = c;
            }
        }

        @Override
        public boolean isFinished() {
            return !wheel.hasBall();
        }
}
