import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    SparkMax flyWheelMotor;
    SparkMax turningMotor;
    SparkMaxConfig flyWheelMotorConfig;
    SparkMaxConfig turningMotorConfig;
    AbsoluteEncoder turningEncoder;

    public ShooterSubsystem() {
        flyWheelMotor = new SparkMax(ShooterConstants.flyWheelMotorid, MotorType.kBrushless);
        turningMotor = new SparkMax(ShooterConstants.turningMotorid, MotorType.kBrushless);
        flyWheelMotorConfig = new SparkMaxConfig();
        turningMotorConfig = new SparkMaxConfig();
        turningEncoder = turningMotor.getAbsoluteEncoder();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turning Anlge", turningEncoder.getPosition());
    }
}
