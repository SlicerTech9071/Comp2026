package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class ExpandTest extends Command{
    HopperSubsystem hopper;
    public ExpandTest(HopperSubsystem hopper) {
        this.hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        hopper.runExpansionMotors(true);
    }

    @Override
    public void end(boolean interrupted) {
        hopper.stopExpansion();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
