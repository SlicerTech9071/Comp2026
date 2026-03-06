package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

public class HopperTelop extends Command{
    HopperSubsystem hopper;
    DoubleSupplier indexerSpeed;
    public HopperTelop(HopperSubsystem hopper, DoubleSupplier indexerSpeed){
        this.hopper = hopper;
        this.indexerSpeed = indexerSpeed;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        hopper.runIndexer(indexerSpeed.getAsDouble());
        hopper.holdExpansionMotors(Constants.hopperConstants.hopperHoldVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        hopper.stopExpansion();
        hopper.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
