package org.firstinspires.ftc.teamcode;

/**
 * Created by 8749 on 3/12/2018.
 */

public class toggle {

    private boolean lastrTrig = false;
    private boolean rTrigoutput = false;
    private boolean toggleOutput = false;

    private boolean rTrig(boolean input)
    {
        if (!lastrTrig && input)
        {
            rTrigoutput = true;
        }
        else
        {
            rTrigoutput = false;
        }
        lastrTrig = input;
        return rTrigoutput;

    }

    public boolean value(boolean input)
    {
        if (rTrig(input))
        {
            toggleOutput = !toggleOutput;
        }
        return toggleOutput;

    }

}