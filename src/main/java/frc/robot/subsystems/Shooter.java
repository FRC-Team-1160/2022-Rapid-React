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

  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


  public static Shooter getInstance(){
    if (m_instance == null){
      m_instance = new Shooter();
    }
    return m_instance;
  }
  
  public Shooter() {
    if (Constants.isFinal){
      m_leftShooter = new CANSparkMax(PortConstantsFinal.LEFT_SHOOTER, MotorType.kBrushed);
      m_rightShooter = new CANSparkMax(PortConstantsFinal.RIGHT_SHOOTER, MotorType.kBrushed);

    }else{
      m_leftShooter = new CANSparkMax(PortConstants.LEFT_SHOOTER, MotorType.kBrushed);
      m_rightShooter = new CANSparkMax(PortConstants.RIGHT_SHOOTER, MotorType.kBrushed);
    }

    m_leftEncoder = m_leftShooter.getEncoder(Type.kQuadrature, 1024);
    m_rightEncoder = m_rightShooter.getEncoder(Type.kQuadrature, 1024);

    m_leftShooter.restoreFactoryDefaults();
    m_rightShooter.restoreFactoryDefaults();

    m_rightShooter.follow(m_leftShooter, false);

    m_shootController = m_leftShooter.getPIDController();

    // PID coefficients
    kP = 0.05; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.5; 
    kMinOutput = -0.5;

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

  public void setReference(double input){
    m_shootController.setReference(input, ControlType.kVelocity);
  }

  public void shooterControl(double input){
    m_leftShooter.setVoltage(-input); 
  }

  public void setShooterVelocity(double input){
    m_shootController.setReference(input, ControlType.kVelocity);
  }

  public double getBallVelocity(double distance){
    return -1 * (distance * ShooterConstants.GRAVITY) / (Math.sqrt(2 * ShooterConstants.GRAVITY * ((ShooterConstants.HUB_HEIGHT * Math.cos(ShooterConstants.LIMELIGHT_ANGLE)) - (distance * Math.sin(ShooterConstants.LIMELIGHT_ANGLE)))));
  }

  public double getShooterVelocity(double ballVelocity){
    return ballVelocity * Math.sqrt(((2 * ShooterConstants.BALL_MASS) / (ShooterConstants.SHOOTER_MASS)) + 1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Distance", m_rightEncoder.getPosition());
  }
}