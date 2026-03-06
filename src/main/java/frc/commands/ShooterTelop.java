package frc.commands;

import java.security.PublicKey;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTelop extends Command{
    ShooterSubsystem shooter;
    DoubleSupplier speed;
    public ShooterTelop(ShooterSubsystem shooter, DoubleSupplier speed){
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooter.runFlyWheelMotor(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlyWheel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
