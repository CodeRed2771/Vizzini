package com.coderedrobotics.vizzini;

public class MotionProfileTrapezoidal {
	double m_accel, m_decel, m_max_speed, m_distance; 
	double m_t1, m_t2, m_t3; 

	public MotionProfileTrapezoidal() {
		m_accel = 0.1;		// ft/s/s 
		m_decel = 0.13;		// ft/s/s 
		m_max_speed = 0.5;	// ft/s 
		m_distance = 2.21154;	//ft 
		m_t1 = -99;			// seconds 
		m_t2 = -99;			// seconds 
		m_t3 = -99;			// seconds 
		
		CalcParams(); 
	}
	
	public MotionProfileTrapezoidal(double Accel, double Decel, double MaxSpeed, double Distance) {
		m_accel = Accel; 
		m_decel = Decel; 
		m_max_speed = MaxSpeed; 
		m_distance = Distance;
		m_t1 = -99;			// seconds 
		m_t2 = -99;			// seconds 
		m_t3 = -99;			// seconds 
		
		CalcParams(); 		
	}
	
	public double Position(double curTime) {
		double position = 0.0; 
		if (curTime < m_t1) position = 0.5*m_accel*curTime*curTime; 
		else if (curTime < m_t2) position = 0.5*m_accel*m_t1*m_t1 + (curTime - m_t1)*m_max_speed; 
		else if (curTime <= m_t3) position = 0.5*m_accel*m_t1*m_t1 + (m_t2 - m_t1)*m_max_speed + 0.5*m_decel*(m_t3 - m_t2)*(m_t3 - m_t2) - 0.5*m_decel*(m_t3 - curTime)*(m_t3 - curTime); 
		else position = m_distance; 
			 
		return position; 
	
	}
	
	public void SetAccel (double value){ 
		m_accel = value; 
	} 
	public void SetDecel(double value){ 
		m_decel = value; 
	} 
	public void SetMaxSpeed(double value){ 
		m_max_speed = value; 
	} 
	public void SetDistance(double value){ 
		m_distance = value; 
	} 
	public double GetAccelTime() {
		return m_t1;
	}
	public double GetDecelTime() {
		return m_t3;
	}

	public void CalcParams() {
		double accel_time, decel_time, min_dist, accel_dist, decel_dist; 

		accel_time = m_max_speed / m_accel; 
		decel_time = m_max_speed / m_decel; 
		accel_dist = 0.5*m_accel * accel_time*accel_time; 
		decel_dist = 0.5*m_decel * decel_time*decel_time; 
		min_dist = accel_dist + decel_dist; 

		if (min_dist > m_distance) // never gets up to speed. 
			{ 
			m_t1 = Math.pow(2.0* m_distance/(m_accel + (m_accel*m_accel)/m_decel),0.5); 
			m_t2 = m_t1; // no constant velocity time 
			m_t3 = m_accel/m_decel*m_t1 + m_t1; 
			} 
		else 
			{ 
			m_t1 = accel_time; 
			m_t2 = (m_distance - min_dist) / m_max_speed + m_t1; // time at end of constant velocity 
			m_t3 = decel_time + m_t2; 
			} 
	}

}
