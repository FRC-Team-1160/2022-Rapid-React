/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;


public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private static Intake m_instance;

  private final CANSparkMax m_middleIndexer, m_finalIndexer, m_intake;

  //private final CANEncoder m_intakeAngleEncoder;

  double k;

  public static Intake getInstance(){
    if (m_instance == null){
      m_instance = new Intake();
    }
    return m_instance;
  }

  public Intake() {
    if (Constants.isFinal){
        m_middleIndexer = new CANSparkMax(PortConstantsFinal.MIDDLE_INDEXER, MotorType.kBrushless);
        m_finalIndexer = new CANSparkMax(PortConstantsFinal.FINAL_INDEXER, MotorType.kBrushless);
        m_intake = new CANSparkMax(PortConstantsFinal.INTAKE, MotorType.kBrushless);
    }
    else {
        m_middleIndexer = new CANSparkMax(PortConstantsFinal.MIDDLE_INDEXER, MotorType.kBrushless);
        m_finalIndexer = new CANSparkMax(PortConstantsFinal.FINAL_INDEXER, MotorType.kBrushless);
        m_intake = new CANSparkMax(PortConstantsFinal.INTAKE, MotorType.kBrushless);
    }

    m_middleIndexer.restoreFactoryDefaults();
    m_finalIndexer.restoreFactoryDefaults();
    m_intake.restoreFactoryDefaults();

    // SmartDashboard.putNumber("input number", k);
  }

  public void intakeControl(double input){
    m_intake.setVoltage(input);
  }

  public void intakeControl(){
    m_intake.setVoltage(k);
  }

  public void middleIndexerControl(double input){
    m_middleIndexer.setVoltage(input);
  }

  public void finalIndexerControl(double input){
    m_finalIndexer.setVoltage(input);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // k = SmartDashboard.getNumber("input number", 0.0);
    //SmartDashboard.putNumber("key", m_intakeAngleEncoder.getPosition());
  }
}