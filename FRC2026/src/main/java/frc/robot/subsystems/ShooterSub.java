package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import org.opencv.core.Mat;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.LimelightHelpers;

public class ShooterSub extends SubsystemBase{
    SparkMax flyWheelMotor;
    SparkMax turningMotor;
    SparkMaxConfig flyWheeConfig;
    SparkMaxConfig turningMotorConfig;
    RelativeEncoder turningEncoder;
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    public ShooterSub() {
        flyWheelMotor = new SparkMax(Constants.ShooterConstants.flyWheelMotorid, MotorType.kBrushless);
        turningMotor = new SparkMax(Constants.ShooterConstants.turningMotorid, MotorType.kBrushless);
        flyWheeConfig = new SparkMaxConfig();
        turningMotorConfig = new SparkMaxConfig();
        turningEncoder = turningMotor.getEncoder();

    }

    public double aprilTagDist(double txnc, double tync, int id) {
        if (id == 10){
            //Get distance away from wall
            //Height from camera to april tag / tan(anlge of limelight)
            double d1 = (Constants.AprilTagConstants.AprilTag10.height - Constants.ShooterConstants.limeLightMountHeight)/Math.tan((Constants.ShooterConstants.limeLightMountAngle + tync));
            return d1 * Math.cos(txnc);
        }
        return 0;
    }

    public double getShooterAngle() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        for (RawFiducial fiducial : fiducials){
            double d1 = aprilTagDist(fiducial.txnc, fiducial.tync, fiducial.id);
            if (d1 == 0){
                return 0;
            }
            double C = m_gyro.getAngle()-fiducial.txnc;
            double x = d1*Math.sin(C);
            double y = d1*Math.cos(C);

            double offset_x = Constants.ShooterConstants.limeLightMountDistance*Math.sin(m_gyro.getAngle()) - Constants.ShooterConstants.shooterDistance*Math.sin(m_gyro.getAngle() + Constants.ShooterConstants.shooterAngularOffset); 
            double offset_y = Constants.ShooterConstants.limeLightMountDistance*Math.cos(m_gyro.getAngle()) - Constants.ShooterConstants.shooterDistance*Math.cos(m_gyro.getAngle() + Constants.ShooterConstants.shooterAngularOffset); 

            return Math.atan2((x+offset_x),(y+offset_y+Constants.AprilTagConstants.AprilTag10.distanceToGoal));
        }
        return 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Angle", turningEncoder.getPosition());
    }

}
