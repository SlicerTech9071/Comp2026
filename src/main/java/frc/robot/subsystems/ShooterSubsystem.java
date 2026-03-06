package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AprilTags.AprilTag10;
import frc.robot.Constants.AprilTags.AprilTag2;
import frc.robot.Constants.AprilTags.AprilTag5;
import frc.robot.LimelightHelpers.RawFiducial;

import edu.wpi.first.units.measure.Angle;

import frc.robot.LimelightHelpers;

//ShooterSub controls anything that deals with the shooting of fuel. Aiming, Flywheel, ...
//Theta is the angle that controls the rotating turret. Relative to the feild.
//Alpha is the angle that controls the hood angle/launch angle.
public class ShooterSubsystem extends SubsystemBase {
    public SparkMax flyWheelMotorLeft;
    public SparkMax flyWheelMotorRight;
    // public SparkMax turningMotor;
    // public SparkMax hoodMotor;
    public SparkMaxConfig flyWheelMotorConfig;
    // public SparkMaxConfig turningMotorConfig;
    // public SparkMaxConfig hoodMotorConfig;

    RelativeEncoder flyWheelEncoder;
    // AbsoluteEncoder turningEncoder;
    // RelativeEncoder hoodMotorEncoder;

    PIDController flyWheelPID;
    // PIDController turningPID;
    // PIDController hoodMotorPID;

    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
    public ShooterSubsystem() {
        flyWheelMotorLeft = new SparkMax(ShooterConstants.flyWheelMotorLeftid, MotorType.kBrushless);
        flyWheelMotorRight = new SparkMax(ShooterConstants.flyWheelMotorRightid, MotorType.kBrushless);
        // turningMotor = new SparkMax(ShooterConstants.turningMotorid, MotorType.kBrushless);
        // hoodMotor = new SparkMax(ShooterConstants.hoodMotorid, MotorType.kBrushless);
        flyWheelMotorConfig = new SparkMaxConfig(); 
        // turningMotorConfig = new SparkMaxConfig();
        // hoodMotorConfig = new SparkMaxConfig();

        flyWheelMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50);
        // turningMotorConfig
        // .inverted(false)
        // .idleMode(IdleMode.kBrake)
        // .smartCurrentLimit(50);
        // hoodMotorConfig
        // .inverted(false)
        // .idleMode(IdleMode.kBrake)
        // .smartCurrentLimit(50);

        flyWheelMotorLeft.configure(flyWheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flyWheelMotorRight.configure(flyWheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flyWheelEncoder = flyWheelMotorLeft.getEncoder();
        // turningEncoder = turningMotor.getAbsoluteEncoder();
        // hoodMotorEncoder = hoodMotor.getEncoder();

        flyWheelPID = new PIDController(ShooterConstants.flyWheelkP, 0, 0);
        // turningPID = new PIDController(ShooterConstants.turningkP, 0, 0);
        // hoodMotorPID = new PIDController(ShooterConstants.hoodkP, 0,0);

        flyWheelPID.setTolerance(ShooterConstants.flyWheelError);
        // turningPID.setTolerance(ShooterConstants.turningError);
        // hoodMotorPID.setTolerance(ShooterConstants.hoodError);

        SmartDashboard.putData("FlyWheelPid",flyWheelPID);
    }

    //Finds the distance away from and fidicual the limelight finds
    public double yDistanceToFidicual(double tync, double fidicualHeight) {
        double h = fidicualHeight - ShooterConstants.limelightHeight;
        double angle = Math.tan(Degrees.of(tync).in(Radians) + ShooterConstants.limelightMountAngle.in(Radians));
        return h/angle;   
    }

    //Finds the left or right distance from any fidicual
    public double xDistanceToFidicual(double txnc, double dy, double gyroAngle) {
        return dy * Math.tan(gyroAngle - txnc);
    }

    //Finds the offset of the Camera and Shooter in the normal x and normal y
    //Important for calculating angle theta
    public double xCameraShooterOffset(double gyroAngle) {
        return ShooterConstants.limelightDistanceCenter*Math.sin(gyroAngle) - ShooterConstants.shooterDistanceCenter*Math.sin(gyroAngle+ShooterConstants.shooterAngleOffset.in(Radians));
    }
    public double yCameraShooterOffset(double gyroAngle) {
        return ShooterConstants.limelightDistanceCenter*Math.cos(gyroAngle) - ShooterConstants.shooterDistanceCenter*Math.cos(gyroAngle+ShooterConstants.shooterAngleOffset.in(Radians));
    }

    //Finds angle theta. There is defintaly a cleaner way to do this.
    public double shooterAngleToTarget() {
        try {         
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(ShooterConstants.limelightName);

            double o_x;
            double o_y;
            double d_y;
            double d_x;

            double aprilTagHeight = 0;
            double aprilTagXDis = 0;
            double aprilTagYDis = 0;

            for (RawFiducial fiducial : fiducials){
                switch (fiducial.id) {
                    case 2:
                        aprilTagHeight = AprilTag2.height;
                        aprilTagXDis = AprilTag2.xDis;
                        aprilTagYDis = 0;
                        break;
                    case 5:
                        aprilTagHeight = AprilTag5.height;
                        aprilTagXDis = AprilTag5.xDis;
                        aprilTagYDis = 0;
                        break;
                    case 10:
                        aprilTagHeight = AprilTag10.height;
                        aprilTagXDis = 0;
                        aprilTagYDis = AprilTag10.yDis;
                        break;
                    default:
                        System.out.println("Could not find fiducial/AprilTag");
                        break;
                }
                if (fiducial.id == 2 || fiducial.id == 5 || fiducial.id == 10){
                    o_x = xCameraShooterOffset(getAngle().in(Radians));
                    o_y = yCameraShooterOffset(getAngle().in(Radians));

                    d_y = yDistanceToFidicual(fiducial.tync, aprilTagHeight);
                    d_x = xDistanceToFidicual(fiducial.txnc, d_y, getAngle().in(Radians));

                    return Math.atan2(d_x + o_x + aprilTagXDis, d_y + o_y + aprilTagYDis);
                }
            }

        } catch (Exception e){
            System.out.println("LimelightError: ShooterSub Line: 59");
        }
        return 0;
    }
    
    //Adjust theta such that the sum of the robot vel and fuels shooting vel equal theta
    //Acounts theta for the robot velocity
    public double adjustAngleForRobotSpeed(double theta, double robotXVel, double robotYVel, double ballSpeed) {
        return theta - Math.asin(((robotXVel + Math.tan(theta)*robotYVel)*Math.cos(theta))/ballSpeed);
    }

    //Combines ever theta function to find the corret angle for the turret
    public double calcTurningAngle() {
        double ballVelo = ShooterConstants.ballVelo;
        double theta = shooterAngleToTarget();
        theta = adjustAngleForRobotSpeed(theta, m_gyro.getVelocityX(), m_gyro.getVelocityY(), ballVelo);
        return theta;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance", yDistanceToFidicual(LimelightHelpers.getTYNC(""), 0.457));
        SmartDashboard.putNumber("FlyWheel RPM", flyWheelEncoder.getVelocity());
    }
    //Returns robot angle
    public Angle getAngle() {
        Angle angle = Degrees.of(m_gyro.getAngle());
        return angle;
    }
    
    //Angle of turret with respect to the feild
    // public Angle getTurretAngleFeild() {
    //     Angle angle = Degrees.of(m_gyro.getAngle() + turningEncoder.getPosition());
    //     return angle;
    // }

    // //Manual control of Turning Motor
    // public void runTurningingMotor(double speed) {
    //     turningMotor.set(speed);
    // }
    
    // //PID control of Turning Motor
    // public void setTurningPos(double angle) {
    //     turningPID.setSetpoint(angle);
    // }
    // public void turningMoveTo(){
    //     double speed = MathUtil.clamp(turningPID.calculate(getTurretAngleFeild().in(Degrees)), -1, 1);
    //     runTurningingMotor(speed);
    // }

    public void teleop() {

    }

    //Manual control of FlyWheel Motor
    public void runFlyWheelMotor(double speed) {
        flyWheelMotorLeft.set(speed);
        flyWheelMotorRight.set(-speed);
    }

    //PID control of FlyWheel Motor
    public void setRPM(double RPM) {
        if (RPM < 4000){
            flyWheelPID.setSetpoint(RPM);
        }
    }
    public void runFlyWheel() {
        double output = flyWheelPID.calculate(flyWheelEncoder.getVelocity()) + ShooterConstants.flyWheelfeedFoward;
        output = MathUtil.clamp(output, 0, 1);
        runFlyWheelMotor(output);
    }

    public void stopFlyWheel() {
        flyWheelMotorLeft.set(0);
        flyWheelMotorRight.set(0);
    }

    // //Manual Controll of Hood Motor
    // public void runHoodMotor(double speed) {
    //     hoodMotor.set(speed);
    // }

    // //PID control of Hood Motor
    // public void setPointHoodPID(double angle) {
    //     hoodMotorPID.setSetpoint(angle);
    // }
    // public void hoodMoveTo(){
    //     double output = hoodMotorPID.calculate(hoodMotorEncoder.getPosition());
    //     output = MathUtil.clamp(output,-1,1);
    //     runHoodMotor(output);
    // }

    //Checks if PID controlls are within threshold
    // public boolean readyToFire() {
    //     if (flyWheelPID.atSetpoint() || turningPID.atSetpoint() || hoodMotorPID.atSetpoint()) {
    //         return true;
    //     } else {
    //         return false;
    //     }
    // }
}
