package org.jumprobotics.gyro;

import static frc.robot.Constants.*;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;

public class GyroIOPigeon implements GyroIO {
  private final Pigeon2 gyro;
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon(int id) {
    gyro = new Pigeon2(id, CAN_BUS_NAME);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    gyro.getRawGyro(xyzDps);
    inputs.connected = gyro.getLastError().equals(ErrorCode.OK);
    inputs.positionDeg = gyro.getYaw(); // degrees
    inputs.velocityDegPerSec = xyzDps[2]; // degrees per second
    inputs.pitchPosition=gyro.getPitch();
  }
}
