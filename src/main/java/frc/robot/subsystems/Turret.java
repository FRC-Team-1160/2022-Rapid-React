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

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */  
  private static Turret m_instance;

  private CANSparkMax m_turret;

  public static Turret getInstance(){
    if (m_instance == null){
      m_instance = new Turret();
    }
    return m_instance;
  }

  public Turret() {
    if (Constants.isFinal){
        m_turret = new CANSparkMax(PortConstantsFinal.TURRET, MotorType.kBrushless);
    }else{
        m_turret = new CANSparkMax(PortConstants.TURRET, MotorType.kBrushless);
    }

    m_turret.restoreFactoryDefaults();
  }

  /**
   * Controls the turret motor.
   */

  public void turretControl(double speed){
    m_turret.setVoltage(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}