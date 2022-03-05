/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */  
  private static Turret m_instance;

  private CANSparkMax m_turret;

  private double lastTurnNumber;

  public PIDController m_turnController;

  public AHRS m_gyro;

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

    m_turnController = new PIDController(AutoConstants.TURN_KP, AutoConstants.TURN_KI, AutoConstants.TURN_KD);
    m_turnController.enableContinuousInput(AutoConstants.MIN_INPUT, AutoConstants.MAX_INPUT);
    m_turnController.setIntegratorRange(AutoConstants.MIN_INGL, AutoConstants.MAX_INGL);
    m_turnController.setTolerance(AutoConstants.TOLERANCE);
    m_turnController.setSetpoint(0);

    lastTurnNumber = 0;

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      m_gyro = new AHRS(Port.kMXP);
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    m_turret.restoreFactoryDefaults();
  }

  /**
   * Controls the turret motor.
   */

  public void turretControl(double speed){
    m_turret.setVoltage(speed);
  }

  public void voltageDrive(double voltage){
    double sign = Math.signum(voltage);
    m_turret.setVoltage(sign*AutoConstants.kS + voltage);
  }

  /**
   * Returns the yaw of the Gyro.
   *
   * @return The yaw.
   */
  public double getYaw() {
    return m_gyro.getYaw();
  }

   /**
   * @param lastTurnNumber the lastTurnNumber to set
   */
  public void setLastTurnNumber(double lastTurnNumber) {
    this.lastTurnNumber = lastTurnNumber;
  }

  /**
   * @return the lastTurnNumber
   */
  public double getLastTurnNumber() {
    return lastTurnNumber;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}