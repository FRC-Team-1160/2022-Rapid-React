/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;

public class Intake extends SubsystemBase{
  /** 
   * Creates a new Intake.
   */
  private static Intake m_instance;

  private final CANSparkMax m_intake;

  public static Intake getInstance(){
    if (m_instance == null){
      m_instance = new Intake();
    }
    return m_instance;
  }

  public Intake() {

    if (Constants.isFinal){
      m_intake = new CANSparkMax(PortConstantsFinal.INTAKE, MotorType.kBrushed);

    }else{
      m_intake = new CANSparkMax(PortConstants.INTAKE, MotorType.kBrushed);
    }
    
    m_intake.restoreFactoryDefaults();

  }

  public void intakeControl(double input){
    m_intake.setVoltage(input);
  }
  
  @Override
  public void periodic() {
    //called oncer per run
  }
}