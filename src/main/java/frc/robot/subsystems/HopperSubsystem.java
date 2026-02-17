package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.hopperConstants;

//HopperSub control any storing and indexing of any fuel
public class HopperSubsystem extends SubsystemBase{
     SparkMax hopperIndexerMotor;
     SparkMax expansionMotor;
     
     RelativeEncoder expansionEncoder;

     SparkMaxConfig hopperIndexerConfig;
     SparkMaxConfig expansionConfig;
    
     DigitalInput limitSwitch;

     public HopperSubsystem() {
        hopperIndexerMotor = new SparkMax(hopperConstants.indexerMotorid, MotorType.kBrushless);
        expansionMotor = new SparkMax(hopperConstants.expansionMotorid, MotorType.kBrushless);

        hopperIndexerConfig = new SparkMaxConfig();
        expansionConfig = new SparkMaxConfig();
    
        expansionEncoder = expansionMotor.getEncoder();
        
        hopperIndexerMotor.configure(hopperIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        expansionMotor.configure(expansionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        limitSwitch = new DigitalInput(HopperConstants.hopperLimitid);
    }

    @Override
    public void periodic() {

    }

    public void extendHopper() {
        if (limitSwitch.get()) {
            expansionMotor.set(HopperConstants.hopperExtensionSpeed);
        }
    }

    public void teleop() {
        hopperIndexerMotor.set(HopperConstants.indexerSpeed);
    }

}
