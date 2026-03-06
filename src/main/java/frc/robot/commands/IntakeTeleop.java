package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTeleop extends Command{
    IntakeSubsystem intake;
    DoubleSupplier speed;
    public IntakeTeleop(IntakeSubsystem intake, DoubleSupplier speed){
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intake.runIntake(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
