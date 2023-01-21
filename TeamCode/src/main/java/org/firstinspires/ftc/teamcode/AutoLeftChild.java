package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// Andrew Test
@Autonomous(name = "AutoLeftChild", group = "ConceptBlue")
public class AutoLeftChild extends AutoParent{
    public double adjustTurn(double angle){
       return angle;
    }
    public double adjustTrajectorydistance(double distance){
        return distance;
    }
    protected void park() {
        if (targetZone == 3) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(137)));
            back(23);

        } else if (targetZone == 2) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(137)));
        } else if (targetZone == 1) {
            back(9);
            odoDriveTrain.turn(Math.toRadians(adjustTurn(150)));
            forward(26);

        }
    }

}
