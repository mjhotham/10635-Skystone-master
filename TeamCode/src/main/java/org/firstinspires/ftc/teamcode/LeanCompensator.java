package org.firstinspires.ftc.teamcode;

public class LeanCompensator {

    public static double choose(double n, double x1, double x2, double y1, double y2) {
        return Math.abs(n-x1) < Math.abs(n-x2) ? y1 : y2;
    }

    public static double getTheThing(double input) {   //great name
        input *= 1.0;
        final double[] inputs = RobotConstants.LeanLiftPositions;
        final double[] outputs = RobotConstants.LeanTopSlidePositions;

        if (inputs.length != outputs.length)
            try {
                throw new Exception("the number of inputs and outputs for the slide/lift tilt compensator need to match");
            } catch (Exception e) {
                e.printStackTrace();
            }

        if (input <= inputs[0])
            return outputs[0];
        else if (input > inputs[inputs.length - 1])
            return outputs[outputs.length - 1];

        int index = 0;
        for (; index < inputs.length; index++)
            if (input < inputs[index])
                break;

        return choose(input, inputs[index - 1], inputs[index], outputs[index - 1], outputs[index]);
    }
}
