package org.firstinspires.ftc.teamcode;
public class rateLimiter
{
	private double outputRateLimiter;
	private double outputFilter;

	public double ratelimiter(double input, double ratelimit)
	{
		ratelimit = Math.abs(ratelimit); //make the limit absolute, negative limit doesnt mean anything to me
		if (Math.abs(outputRateLimiter - input) > ratelimit)
		{
			if (input < outputRateLimiter)
			{
				outputRateLimiter = outputRateLimiter - ratelimit;
			}
			else if (input > outputRateLimiter)
			{
				outputRateLimiter = outputRateLimiter + ratelimit;
			}

		}

		else
		{
			outputRateLimiter = input;
		}

		return outputRateLimiter;
	}

	public double filter(double input, double filterConstant)
	{
		filterConstant = Math.abs(filterConstant); // prevent negative
		if (filterConstant > 100)
		{
			filterConstant = 100; // limit to 100
		}
		outputFilter = outputFilter * ((100 - filterConstant) / 100) + input * (filterConstant / 100); //store output to intermediate variable so we can use it next scan
		return outputFilter;
	}

	public double filteredRateLimit(double input, double ratelimit, double filterConstant)
	{
		return filter(ratelimiter(input, ratelimit), filterConstant);
	}
}