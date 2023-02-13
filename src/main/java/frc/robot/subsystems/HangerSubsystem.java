// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase {
  /** Creates a new HangerSubsystem. */

  TalonFX hanger;
  HangerState hangerState;

  public HangerSubsystem() {
    this.hanger = new TalonFX(3);
    this.hanger.configFactoryDefault();
    this.hanger.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,30);
    this.hanger.setSensorPhase(true);
    this.hanger.setInverted(false);
    this.hanger.configNominalOutputForward(0);
    this.hanger.configNominalOutputReverse(0);
    this.hanger.configPeakOutputForward(0.05);
    this.hanger.configPeakOutputReverse(-0.05);
    this.hanger.configAllowableClosedloopError(0,0,30);
    this.hanger.config_kF(0, 0); 
    this.hanger.config_kP(0, 0.15);
    this.hanger.config_kI(0, 0);
    this.hanger.config_kD(0, 12);

    this.hanger.setSelectedSensorPosition(0);

    this.hangerState = HangerState.ZERO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHangerState(HangerState hState)
  {
    this.hangerState = hState;
  }

  public HangerState getHangerState()
  {
    return this.hangerState;
  }
  
  public void enumHanger(HangerState hangerState)
  {
    this.hanger.set(TalonFXControlMode.Position,hangerState.hangerSetpoint);
    setHangerState(hangerState);
  }

  public enum HangerState
      {
        ZERO(Constants.HangerConstants.DEFAULT),
        CUBE_LOW(Constants.HangerConstants.CUBE_LOW),
        CUBE_MID(Constants.HangerConstants.CUBE_MID),
        CUBE_HIGH(Constants.HangerConstants.CUBE_HIGH),
        CONE_LOW(Constants.HangerConstants.CONE_LOW),
        CONE_MID(Constants.HangerConstants.CONE_MID),
        CONE_HIGH(Constants.HangerConstants.CONE_HIGH);

        public final int hangerSetpoint;
        private HangerState(int setpoint)
        {
          this.hangerSetpoint = setpoint;
        }
      }
}
