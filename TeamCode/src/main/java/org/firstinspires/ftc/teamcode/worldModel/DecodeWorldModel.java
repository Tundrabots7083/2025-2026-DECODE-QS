package org.firstinspires.ftc.teamcode.worldModel;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class DecodeWorldModel extends WorldModel{
    public DecodeWorldModel(){
        init();
    }
    private void init(){
        /// RED ALLIANCE ////////////////////////////////
        /// Red Alliance Goal
        WorldObject redAllianceGoal = new WorldObject("RedAllianceGoal","RedAllianceGoal", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));

        /// Red Alliance Spikes
        WorldObject redAllianceAudienceSpike = new WorldObject("RedAllianceAudienceSpike","RedAllianceAudienceSpike", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));
        WorldObject redAllianceMiddleSpike = new WorldObject("RedAllianceMiddleSpike","RedAllianceMiddleSpike", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));
        WorldObject redAllianceFrontSpike = new WorldObject("RedAllianceFrontSpike","RedAllianceFrontSpike", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));

        /// Red Alliance Gate
        WorldObject redAllianceGate = new WorldObject("RedAllianceGate","RedAllianceGate", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));

        this.setValue(redAllianceGoal);

        this.setValue(redAllianceAudienceSpike);
        this.setValue(redAllianceMiddleSpike);
        this.setValue(redAllianceFrontSpike);

        this.setValue(redAllianceGate);


        ///  END RED ALLIANCE ////////////////////////////


        /// BLUE ALLIANCE ////////////////////////////////
        /// Blue Alliance Goal
        WorldObject blueAllianceGoal = new WorldObject("BlueAllianceGoal","BlueAllianceGoal", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));

        /// Blue Alliance Spikes
        WorldObject blueAllianceAudienceSpike = new WorldObject("BlueAllianceAudienceSpike","BlueAllianceAudienceSpike", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));
        WorldObject blueAllianceMiddleSpike = new WorldObject("BlueAllianceMiddleSpike","BlueAllianceMiddleSpike", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));
        WorldObject blueAllianceFrontSpike = new WorldObject("BlueAllianceFrontSpike","BlueAllianceFrontSpike", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));

        /// Blue Alliance Gate
        WorldObject blueAllianceGate = new WorldObject("BlueAllianceGate","BlueAllianceGate", new Position(DistanceUnit.INCH,25.0, 40.0,0.0,10), new WorldObjectSize(20.0, 41.0));

        this.setValue(blueAllianceGoal);

        this.setValue(blueAllianceAudienceSpike);
        this.setValue(blueAllianceMiddleSpike);
        this.setValue(blueAllianceFrontSpike);

        this.setValue(blueAllianceGate);


        ///  END BLUE ALLIANCE ////////////////////////////
    }
}
