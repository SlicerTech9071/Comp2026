package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.hal.simulation.RelayDataJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.hopperConstants;
import pabeles.concurrency.ConcurrencyOps.Reset;

 public class HopperSubsystem extends SubsystemBase{
     SparkMax hopperIndexerMotor;
     SparkMax expansionMotor;
     
     RelativeEncoder expansionEncoder;

     SparkMaxConfig hopperIndexerConfig;
     SparkMaxConfig expansionConfig;
    
     public HopperSubsystem() {
        hopperIndexerMotor = new SparkMax(hopperConstants.indexerMotorid, MotorType.kBrushless);
        expansionMotor = new SparkMax(hopperConstants.expansionMotorid, MotorType.kBrushless);

        hopperIndexerConfig = new SparkMaxConfig();
        expansionConfig = new SparkMaxConfig();
    
        expansionEncoder = expansionMotor.getEncoder();
        
        hopperIndexerMotor.configure(hopperIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        expansionMotor.configure(expansionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    }
}
