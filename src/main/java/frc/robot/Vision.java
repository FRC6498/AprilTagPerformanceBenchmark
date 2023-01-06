// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.TimestampedRaw;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  PhotonCamera camera;
  NetworkTable pvtable = NetworkTableInstance.getDefault().getTable("photonvision");
  NetworkTable teamtable = NetworkTableInstance.getDefault().getTable("6498");
  RawSubscriber byteSub;
  IntegerPublisher fpsPub;
  IntegerPublisher tagCountPub;
  TimestampedRaw rawBytesCached;
  TimestampedRaw[] byteQueue;
  double deltatime;
  int fps;
  /** Creates a new Vision. */
  public Vision() {
    camera = new PhotonCamera("limelight");
    byteSub = pvtable.getRawTopic("rawBytes").subscribe("rawBytes", new byte[] {}, PubSubOption.sendAll(true), PubSubOption.periodic(0.01));
    fpsPub = teamtable.getIntegerTopic("fps").publish(PubSubOption.periodic(0.02), PubSubOption.sendAll(true));
    tagCountPub = teamtable.getIntegerTopic("tagCount").publish(PubSubOption.periodic(0.02));
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    tagCountPub.set(result.targets.size());
    // get changes from nt
    byteQueue = byteSub.readQueue();
    if (byteQueue.length > 1) {
      // multiple updates since last loop, compare the last two values
      deltatime = byteQueue[byteQueue.length-1].timestamp - byteQueue[byteQueue.length-2].timestamp;
    } else {
      // only one update since last loop
      deltatime = byteQueue[0].timestamp - rawBytesCached.timestamp;
    }
    deltatime /= 1e+6;
    fps = (int)(1/deltatime);
    // update cached value
    rawBytesCached = byteQueue[byteQueue.length-1];
  }
}
