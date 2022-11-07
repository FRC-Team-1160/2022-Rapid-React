/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.*;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private static Shooter m_instance;

  private final CANSparkMax m_leftShooter, m_rightShooter;

  private RelativeEncoder m_leftEncoder, m_rightEncoder;

  private SparkMaxPIDController m_shootController;

  private PIDController m_RIOshootController;

  private double accumulator;

  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, vA, maxA;

  public static Shooter getInstance(){
    if (m_instance == null){
      m_instance = new Shooter();
    }
    return m_instance;
  }
  
  private Shooter() {
    if (Constants.isFinal){
      m_leftShooter = new CANSparkMax(PortConstantsFinal.LEFT_SHOOTER, MotorType.kBrushed);
      m_rightShooter = new CANSparkMax(PortConstantsFinal.RIGHT_SHOOTER, MotorType.kBrushed);

    }else{
      m_leftShooter = new CANSparkMax(PortConstants.LEFT_SHOOTER, MotorType.kBrushed);
      m_rightShooter = new CANSparkMax(PortConstants.RIGHT_SHOOTER, MotorType.kBrushed);
    }

    m_leftEncoder = m_leftShooter.getEncoder(Type.kQuadrature, 1024);
    m_rightEncoder = m_rightShooter.getEncoder(Type.kQuadrature, 1024);

    //m_leftEncoder.setVelocityConversionFactor(1/3.14);
    //m_rightEncoder.setVelocityConversionFactor(1/3.14);
    m_leftEncoder.setPositionConversionFactor(1/3);
    m_rightEncoder.setPositionConversionFactor(1/3);

    m_leftShooter.restoreFactoryDefaults();
    m_rightShooter.restoreFactoryDefaults();

    m_rightShooter.follow(m_leftShooter, false);

    m_shootController = m_leftShooter.getPIDController();

    // PID coefficients
    kP = 0.15; 
    kI = 0;
    kD = 0;
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.5; 
    kMinOutput = -0.5;
    maxA = 30000;
    accumulator = 0;

    //new PID coefficients
    kP = 0.000485;
    kI = 0.0000127;
    kFF = 0.52;
    vA = 4.7;
    

    m_shootController.setP(kP);
    m_shootController.setI(kI);
    m_shootController.setD(kD);
    m_shootController.setIZone(kIz);
    m_shootController.setFF(kFF);
    m_shootController.setOutputRange(kMinOutput, kMaxOutput);

    /*
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Set Rotations", 0);
    */

  }

  public void setAccumulator(double input) {
    accumulator = input;
  }

  public void PIDControl(double input) {
    //the higher the distance, the greater the vA coefficient. equation is y = 0.125(x-3)
    double vCoef = vA + (input*-0.78);

    //PID
    double velocity = vCoef*getShooterVelocity(getBallVelocity(input));
    double error = velocity - m_rightEncoder.getVelocity()/3;

    if (error < 500 && error > -500) {
      accumulator += error;
    }

    if (accumulator > maxA) {
      accumulator = maxA;
    } else if (accumulator < -maxA) {
      accumulator = -maxA;
    }

    double initialVoltage = kFF;
    double normalizedValue = (kI * accumulator) + (kP * error) + initialVoltage;
    //CAP at 1 or -1
    if (normalizedValue > 1) {
      normalizedValue = 1;
    } else if (normalizedValue < -1) {
      normalizedValue = -1;
    }
    double output = normalizedValue * 12;

    m_leftShooter.setVoltage(output);

    SmartDashboard.putNumber("Goal Velocity", velocity);
    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("vCoef", vCoef);
    
    SmartDashboard.putNumber("Accumulator", accumulator);
    SmartDashboard.putNumber("Encoder Velocity", m_leftEncoder.getVelocity());
  }

  public void setReference(double input){
    m_shootController.setReference(input, ControlType.kVelocity);
  }

  public void shooterControl(double input){
    m_leftShooter.setVoltage(-input); 
  }

  public double getBallVelocity(double distance){
    return Math.sqrt((ShooterConstants.GRAVITY*distance*distance)/(2*Math.cos(ShooterConstants.SHOOTER_ANGLE)*Math.cos(ShooterConstants.SHOOTER_ANGLE)*((ShooterConstants.HUB_HEIGHT - ShooterConstants.SHOOTER_HEIGHT) - (Math.tan(ShooterConstants.SHOOTER_ANGLE)*distance))));
    //return -1 * (distance * ShooterConstants.GRAVITY) / (Math.sqrt(2 * ShooterConstants.GRAVITY * ((ShooterConstants.HUB_HEIGHT * Math.cos(ShooterConstants.LIMELIGHT_ANGLE)) - (distance * Math.sin(ShooterConstants.LIMELIGHT_ANGLE)))));
  }

  public double getShooterVelocity(double ballVelocity){
    return ((ballVelocity * Math.sqrt(((2 * ShooterConstants.BALL_MASS) / (ShooterConstants.SHOOTER_MASS)) + 1))/(2*Math.PI*ShooterConstants.WHEEL_RADIUS))*60;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Shooter Velocity Right", m_rightEncoder.getVelocity()/3);
    SmartDashboard.putNumber("Shooter Velocity Left", m_leftEncoder.getVelocity()/3);
    SmartDashboard.putNumber("Shooter Distance", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Bus Voltage", m_leftShooter.getBusVoltage());
    SmartDashboard.putNumber("Ball Velocity", getBallVelocity(Vision.getDistance(Vision.getTy() * Math.sin(ShooterConstants.LIMELIGHT_ANGLE))));
    SmartDashboard.putNumber("Hub Distance", Vision.getDistance(Vision.getTy()));
  }
}