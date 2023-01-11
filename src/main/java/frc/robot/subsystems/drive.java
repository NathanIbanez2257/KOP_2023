// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class drive extends SubsystemBase {

  WPI_TalonFX rightFront = new WPI_TalonFX(Constants.rightFront);
  WPI_TalonFX rightRear = new WPI_TalonFX(Constants.rightRear);
  WPI_TalonFX leftFront = new WPI_TalonFX(Constants.leftFront);
  WPI_TalonFX leftRear = new WPI_TalonFX(Constants.leftRear);

  WPI_TalonFX rightMotor = new WPI_TalonFX(DriveConstants.rightMotorID);
  WPI_TalonFX leftMotor = new WPI_TalonFX(DriveConstants.leftMotorID);

  MotorControllerGroup rightDrive = new MotorControllerGroup(rightFront, rightRear);
  MotorControllerGroup leftDrive = new MotorControllerGroup(leftFront, leftRear);

  public static final WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.PigeonID);

  Rotation2d gyro2D = new Rotation2d(Units.degreesToRadians(getHeading() ));

  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);


  public ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);

  DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds);

  
  public double leftVelocity = wheelSpeeds.leftMetersPerSecond,
         rightVelocity = wheelSpeeds.rightMetersPerSecond;
  

        

  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new drive. */
  public drive() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setSelectedSensorPosition(0);
    rightMotor.setSelectedSensorPosition(0);

 

    setBreakMode();

    //rightMotor.configSelectedFeedbackCoefficient(1/DriveConstants.kGearRatio);
    //leftMotor.configSelectedFeedbackCoefficient(1/DriveConstants.kGearRatio);

    
    

    

    // leftMotor.configSelectedFeedbackCoefficient(1/DriveConstants.kGearRatio);

    // 4277.55248642 counts = 1 meter

    // leftMotor.set(TalonFXControlMode.Position,
    // DriveConstants.kLinearDistanceConversionFactor);
    // rightMotor.set(TalonFXControlMode.Position,
    // DriveConstants.kLinearDistanceConversionFactor);
    /*
     * leftMotor.set(TalonFXControlMode.Velocity,
     * DriveConstants.kLinearDistanceConversionFactor/600);
     * rightMotor.set(TalonFXControlMode.Velocity,
     * DriveConstants.kLinearDistanceConversionFactor/600);
     */

    /*
     * rightRear.follow(rightFront);
     * leftRear.follow(leftFront);
     */

    gyro.setYaw(0);
    gyro.configFactoryDefault();
    leftMotor.setInverted(true);
    
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftNativeDistanceInMeters(leftMotor.getSelectedSensorPosition()), 
    rightNativeDistanceInMeters(rightMotor.getSelectedSensorPosition()));

    m_odometry.resetPosition(gyro2D, 0, 0, getPose());

  }

  public void arcadeDrive(double leftSpeed, double rightSpeed)
  {
    leftMotor.setInverted(true);
    drive.arcadeDrive(leftSpeed, rightSpeed);

  }

  public void move(double leftSpeed, double rightSpeed) {
    leftMotor.setInverted(true);
    drive.tankDrive(leftSpeed, rightSpeed);

  

  }

  /////////////////////////// Motor Modes /////////////////////////
  public void setBreakMode() {
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
    leftMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setNeutralMode(NeutralMode.Coast);
  }

  /////////////////// Resets ////////////////////////
  public void resetEncoders() {
    rightMotor.setSelectedSensorPosition(0);
    leftMotor.setSelectedSensorPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(gyro2D, 0, 0, pose);
  }

  public static void zeroHeading() {
    gyro.setYaw(0);
    gyro.configFactoryDefault();
  }

  public WPI_Pigeon2 getPigeon() {
    return getPigeon();
  }

  ///////////////////// Encoder Position //////////////////////////
  public double getRightEncoderPostion() {
    return -rightMotor.getSelectedSensorPosition();
  }

  public double getLeftEncoderPostion() {
    return -leftMotor.getSelectedSensorPosition();
  }

  public double getRightEncoderVelocity() {
    return leftVelocity;
    }

  public double getLeftEncoderVelocity() {
    return rightVelocity;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }
  /**
   * this is the one being used for the path planning thing
   * @param leftVolts
   * @param rightVolts
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(-leftVolts);
    rightMotor.setVoltage(rightVolts);
    drive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public double getAvgEncoderDistance() {
    return ((getRightEncoderPostion() + getLeftEncoderPostion()) / 2.0);

  }

  public double getRate() {
    return gyro.getRate(); // negative
  }


  public double leftNativeDistanceInMeters(double sensorCounts)
  {
    double motorRotations = leftMotor.getSelectedSensorPosition() / DriveConstants.kCountsPerRev;
    double wheelRotations  = motorRotations / DriveConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));

    return -positionMeters;

  }

  public double rightNativeDistanceInMeters(double sensorCounts)
  {
    double motorRotations = rightMotor.getSelectedSensorPosition() / DriveConstants.kCountsPerRev;
    double wheelRotations  = motorRotations / DriveConstants.kGearRatio;

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));

    return -positionMeters;

  }

  
  /*
   * public double getTurnRate()
   * {
   * }
   */
/**
 * gets the gyro Yaw in degrees
 * @return
 */
  public static double getHeading() 
  {
    return gyro.getYaw();
  }

  @Override
  public void periodic() {
    
    
    Rotation2d gyroAngle = gyro.getRotation2d();
    SmartDashboard.putNumber("test gyro", gyroAngle.getDegrees());
    m_odometry.update(gyroAngle, leftNativeDistanceInMeters(leftMotor.getSelectedSensorPosition()), rightNativeDistanceInMeters(rightMotor.getSelectedSensorPosition()));

    SmartDashboard.putNumber("Pose Meters X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Meters Y", m_odometry.getPoseMeters().getY());

    
    

    
    SmartDashboard.putNumber("Left Encoder Position", leftNativeDistanceInMeters(leftMotor.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Right Encoder Position", rightNativeDistanceInMeters(rightMotor.getSelectedSensorPosition()));


    SmartDashboard.putNumber("Right Encoder Velocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("Left Encoder Velocity", getLeftEncoderVelocity());

    SmartDashboard.putNumber("Inches Conversion", DriveConstants.kxConversion);
    SmartDashboard.putNumber("Meter Conversion", DriveConstants.kMeterConversion);
    SmartDashboard.putNumber("Meter Accurate", DriveConstants.kMeterConversionAccurate);


  }
}
