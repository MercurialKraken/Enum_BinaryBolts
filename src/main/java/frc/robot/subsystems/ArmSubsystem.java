// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  TalonFX arm;
  CANCoder CancoderArm;
  ArmState armState;
  
  public ArmSubsystem() {
    this.arm =new TalonFX(1);
    this.arm.configFactoryDefault();
    this.arm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,30);
    this.arm.setSensorPhase(true);
    this.arm.setInverted(true);
    this.arm.configNominalOutputForward(0);
    this.arm.configNominalOutputReverse(0);
    this.arm.configPeakOutputForward(1);
    this.arm.configPeakOutputReverse(-1);
    this.arm.configAllowableClosedloopError(0,0,30);
    this.arm.config_kF(0, 0);
    this.arm.config_kP(0, 2);
    this.arm.config_kI(0, 0);
    this.arm.config_kD(0, 12);

    this.CancoderArm = new CANCoder(10);
    this.CancoderArm.configFactoryDefault();
    this.CancoderArm.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    this.arm.setSelectedSensorPosition(33.54-CancoderArm.getAbsolutePosition()*2275.5);

    this.armState = ArmState.ZERO;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmState (ArmState aState)
  {
    this.armState = aState;
  }

  public ArmState getArmState ()
  {
    return this.armState;
  }

  public void enumArm(ArmState armState)
      {
        this.arm.set(TalonFXControlMode.Position, ((armState.armSetpoint/(100*3.22))*2048)*50);
        setArmState(armState);
      }

  public enum ArmState
      {
        ZERO(Constants.ArmConstants.DEFAULT),
        CUBE_LOW(Constants.ArmConstants.CUBE_LOW),
        CUBE_MID(Constants.ArmConstants.CUBE_MID),
        CUBE_HIGH(Constants.ArmConstants.CUBE_HIGH),
        CONE_LOW(Constants.ArmConstants.CONE_LOW),
        CONE_MID(Constants.ArmConstants.CONE_MID),
        CONE_HIGH(Constants.ArmConstants.CONE_HIGH);

        public final int armSetpoint;
        private ArmState(int setpoint)
        {
          this.armSetpoint = setpoint;
        }
      }
}
