package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.hopperConstants;

public class HopperSubsystem extends SubsystemBase{
    public SparkMax hopperIndexerMotor;
    public SparkMax expansionMotor;

    public SparkMaxConfig hopperIndexerConfig;
    public SparkMaxConfig expansionConfig;
    
    public HopperSubsystem() {
        hopperIndexerMotor = new SparkMax(hopperConstants.indexerMotorid, MotorType.kBrushless);
        expansionMotor = new SparkMax(hopperConstants.expansionMotorid, MotorType.kBrushless);

        hopperIndexerConfig = new SparkMaxConfig();
        expansionConfig = new SparkMaxConfig();
    }
}
