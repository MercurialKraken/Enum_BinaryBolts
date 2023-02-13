// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new GrabberSubsystem. */
  CANSparkMax Intake;
  Solenoid SolGrip;
  Compressor comp;
  GrabberState grabberState;

  public GrabberSubsystem() {
    // SolGrip = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    // comp = new Compressor(PneumaticsModuleType.CTREPCM);
    // Intake = new CANSparkMax(2, MotorType.kBrushless);
    this.grabberState = GrabberState.ZERO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enumGrabber(GrabberState grabberState)
      {
        //Intake.set(grabberState.grabberPercentOutput); //compliant wheels
        //SolGrip.set(grabberState.grabberPercentOutput>0); //solenoid based grabber
        setGrabberState(grabberState);
      }

  public void setGrabberState (GrabberState gState)
  {
    this.grabberState = gState;
  }

  public enum GrabberState
      {
        ZERO(Constants.GrabberConstants.STOP),
        CONE_INTAKE(Constants.GrabberConstants.INTAKE_CONE),
        CONE_OUTTAKE(Constants.GrabberConstants.OUTTAKE_CONE),
        CUBE_INTAKE(Constants.GrabberConstants.INTAKE_CUBE),
        CUBE_OUTTAKE(Constants.GrabberConstants.OUTTAKE_CUBE);

        public final double grabberPercentOutput;
        private GrabberState(double p_output)
        {
          this.grabberPercentOutput = p_output;
        }
      }
}
