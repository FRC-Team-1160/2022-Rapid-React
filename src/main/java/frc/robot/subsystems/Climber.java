/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */  
  private static Climber m_instance;

  private CANSparkMax m_leftClimber, m_rightClimber;

  public static Climber getInstance(){
    if (m_instance == null){
      m_instance = new Climber();
    }
    return m_instance;
  }

  public Climber() {
    if (Constants.isFinal){
        m_leftClimber = new CANSparkMax(PortConstants.LEFT_CLIMBER, MotorType.kBrushless);
        m_rightClimber = new CANSparkMax(PortConstants.RIGHT_CLIMBER, MotorType.kBrushless);
    }else{
        m_leftClimber = new CANSparkMax(PortConstantsFinal.LEFT_CLIMBER, MotorType.kBrushless);
        m_rightClimber = new CANSparkMax(PortConstantsFinal.RIGHT_CLIMBER, MotorType.kBrushless);
    }

    m_leftClimber.restoreFactoryDefaults();
    m_rightClimber.restoreFactoryDefaults();
  }

  /**
   * Controls the Climber motor.
   */

  public void climbControl(double speed){
    m_leftClimber.setVoltage(speed);
    m_rightClimber.setVoltage(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}