package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
// import frc.robot.subsystems.PoseEstimator;
    
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final static Joystick driver = new Joystick(0);
    // public PhotonCamera photonCamera = new PhotonCamera("AprilTag");
    public Arm arm;
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final int Lefttrigger =  XboxController.Axis.kLeftTrigger.value;
    // private final JoystickButton LeftTrigger = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    private final int Righttrigger =  XboxController.Axis.kRightTrigger.value;
    private final JoystickButton ArmUp = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton ArmDown = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton HangerUp = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton HangerDown = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton Hanger_down = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton Hanger_up = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final Trigger manualTrigger = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    // private final  POVButton pov = new Joystick(driver,XboxController.Button.
  
    private final double speed = 0.7;
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm a_Arm = new Arm();
    // private final PoseEstimator PoseEstimator = new PoseEstimator(photonCamera, s_Swerve);

 
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis)*speed, 
                () -> -driver.getRawAxis(strafeAxis)*speed, 
                () -> -driver.getRawAxis(rotationAxis)*speed, 
                () -> robotCentric.getAsBoolean()
            )
        );
        this.arm = new Arm();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        ArmUp.whileTrue(new InstantCommand(() -> this.arm.MoveArm(-2)));
        ArmDown.whileTrue(new InstantCommand(() -> this.arm.MoveArm(2)));
        HangerUp.whileTrue(new frc.robot.commands.ManualHangerUp(this.arm));
        HangerDown.whileTrue(new frc.robot.commands.ManualHangerDown(this.arm));
        // // MiddleArm.whileTrue(new frc.robot.commands.MiddleArm(this.arm));
        Hanger_up.onTrue(new frc.robot.commands.HangerUp(this.arm));
        Hanger_down.onTrue(new frc.robot.commands.HangerDown(this.arm));

        // new Trigger(()-> ManualArmUp()).whileTrue(new ArmUp(this.arm));
        // new Trigger(()-> ManualArmDown()).whileTrue(new ArmDown(this.arm));
        // new Trigger(()-> ManualHangerUp()).whileTrue(new HangerUp(this.arm));
        // new Trigger(()-> ManualHangerDown()).whileTrue(new HangerDown(this.arm));
// 
        // new Trigger(()-> dpadButtonLeft()&&!dpadButtonRight()).whileTrue(new RotateInside(this.arm));
        // new Trigger(()-> dpadButtonRight()&&!dpadButtonLeft()).whileTrue(new RotateOutside(this.arm));
        // new Trigger(()-> dpadButtonLeft()).whileTrue(new MiddleArm(this.arm));
       
        // new Trigger(()-> dpadButtonUp()).whileTrue(new GripInside(this.arm));
        // new Trigger(()-> dpadButtonDown()).whileTrue(new GripOutside(this.arm));

      
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve, a_Arm);
    }   

    public static boolean dpadButtonRight() {
        return (driver.getPOV() >= 45 && RobotContainer.driver.getPOV() <= 135);
      }
    
      public static boolean dpadButtonLeft() {
        return (RobotContainer.driver.getPOV() >= 225 && RobotContainer.driver.getPOV() <= 315);
      }
    
      public static boolean  dpadButtonUp() {
        return (RobotContainer.driver.getPOV() >= 315 && RobotContainer.driver.getPOV() < 360)
            || (RobotContainer.driver.getPOV() >= 0 && RobotContainer.driver.getPOV() <= 45);
      }
    
      public static boolean dpadButtonDown() {
        return (RobotContainer.driver.getPOV() >= 135 && RobotContainer.driver.getPOV() <= 225);
      }
    
      public static boolean dpadButtonRight_JoyD() {
        return (RobotContainer.driver.getPOV() >= 45 && RobotContainer.driver.getPOV() <= 135);
      }
    
      public static boolean dpadButtonLeft_JoyD() {
        return (RobotContainer.driver.getPOV() >= 225 && RobotContainer.driver.getPOV() <= 315);
      }
    
      public static boolean dpadButtonUp_JoyD() {
        return (RobotContainer.driver.getPOV() >= 315 && RobotContainer.driver.getPOV() < 360)
            || (RobotContainer.driver.getPOV() >= 0 && RobotContainer.driver.getPOV() <= 45);
      }
    
      public static boolean dpadButtonDown_JoyD() {
        return (RobotContainer.driver.getPOV() >= 135 && RobotContainer.driver.getPOV() <= 225);
      }
      public static boolean ManualArmUp(){
        return (RobotContainer.driver.getRawButton(3) && RobotContainer.driver.getRawButton(8));
      }

      public static boolean ManualArmDown(){
        return (RobotContainer.driver.getRawButton(2) && RobotContainer.driver.getRawButton(8));
      }

      public static boolean ManualHangerUp(){
        return (RobotContainer.driver.getRawButton(4) && RobotContainer.driver.getRawButton(8));
      }

      public static boolean ManualHangerDown(){
        return (RobotContainer.driver.getRawButton(1) && RobotContainer.driver.getRawButton(8));
      }

}
