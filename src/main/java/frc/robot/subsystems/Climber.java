// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;


/** Add your docs here. */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static Climber m_instance;

  private final CANSparkMax m_climber;

  public static Climber getInstance() {
    if (m_instance == null) {
      m_instance = new Climber();
    }
    return m_instance;
  }

  public Climber() {
    if (Constants.isFinal){
      m_climber = new CANSparkMax(PortConstantsFinal.CLIMBER, MotorType.kBrushless);
    }else{
      m_climber = new CANSparkMax(PortConstants.CLIMBER, MotorType.kBrushless);
    }
    m_climber.restoreFactoryDefaults();
  }

  /**
   * Controls the Climber motor.
   */

  public void climbControl(double speed){
    m_climber.setVoltage(speed);
  }
  
}
