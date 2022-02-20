package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
/*
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
*/

// Subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

// Commands
import frc.robot.commands.climb.ClimbControl;
import frc.robot.commands.turret.TurretControl;
import frc.robot.commands.intake.MiddleIndexerControl;
import frc.robot.commands.intake.FinalIndexerControl;
import frc.robot.commands.intake.IntakeControl;
import frc.robot.commands.shoot.ShooterControl;

/*
import frc.robot.commands.drive.Drive;
import frc.robot.commands.vision.LimelightCameraToggle;
import frc.robot.commands.vision.LimelightLightToggle;
import frc.robot.commands.vision.LimelightSnapshotToggle;
import frc.robot.commands.vision.LimelightStreamToggle;
*/

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    public final DriveTrain m_driveTrain = DriveTrain.getInstance(); 
    public final Shooter m_shooter = Shooter.getInstance();
    public final Intake m_intake = Intake.getInstance();
    public final Climber m_climber = Climber.getInstance();
    public final Turret m_turret = Turret.getInstance();
  
    // Controllers
    private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
    private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);
    private Joystick m_secondStick = new Joystick(OIConstants.secondStickPort);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands
      m_driveTrain.setDefaultCommand(new RunCommand(
        () -> m_driveTrain.tankDrive(0.8 * m_mainStick.getRawAxis(1), 0.8 * m_mainStick.getRawAxis(4), 0.8 * m_mainStick.getRawAxis(2), 0.8 * m_mainStick.getRawAxis(3)),
        m_driveTrain)
      );
    }
  
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
      
      // Config for main stick

      // Spin green wheels on arm OUT
      new JoystickButton(m_mainStick, Button.kLeftBumper.value)
        .whileHeld(
          new IntakeControl(m_intake, 0.5 * 12)
        );
      
      // Spin green wheels on arm IN
      new JoystickButton(m_mainStick, Button.kRightBumper.value)
        .whileHeld(
          new IntakeControl(m_intake, -0.4 * 12)//-0.3,-0.5
        );

      // Spin green wheels on middle-inside OUT
      new JoystickButton(m_firstStick, 7)
        .whileHeld(
          new MiddleIndexerControl(m_intake, 0.5 * 12)
        );
      
      // Spin green wheels on middle-inside IN
      new JoystickButton(m_firstStick, 2)
        .whileHeld(
          new MiddleIndexerControl(m_intake, -0.65 * 12)
        );
      
      // Spin green wheels on back-inside OUT
      new JoystickButton(m_firstStick, 6)
        .whileHeld(
          new FinalIndexerControl(m_intake, 0.5 * 12)
        );
      
      // Spin green wheels on back-inside IN
      new JoystickButton(m_firstStick, 3)
        .whileHeld(
          new FinalIndexerControl(m_intake, -0.65 * 12)
        );

      // Shoot at high speed
      new JoystickButton(m_firstStick, 1)
        .whileHeld(
          new ShooterControl(m_shooter, -1 * 12)  // 0.81
        );
      
      // Config for first stick

      // Turn turret complex to the RIGHT
      new JoystickButton(m_firstStick, 5)
        .whenHeld(
          new TurretControl(m_turret, 0.1 * 12)
        );

      // Turn turret complex to the LEFT
      new JoystickButton(m_firstStick, 4)
        .whenHeld(
          new TurretControl(m_turret, -0.1 * 12)
        );
      
      // Climber FIRST DIRECTION
      new JoystickButton(m_firstStick, 11)
        .whenHeld(
          new ClimbControl(m_climber, -0.1 * 12)
        );
      
      // Climber SECOND DIRECTION
      new JoystickButton(m_firstStick, 10)
        .whenHeld(
          new ClimbControl(m_climber, 0.1 * 12)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    /*
    public Command getAutonomousCommand() {
      return m_chooser.getSelected().withTimeout(15);
    }
    */
}