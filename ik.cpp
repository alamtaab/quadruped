#include <iostream>
#include <cmath>

int main() {
	
	float a1 = 7.7f;
	float a2 = 11.5f; // section lengths
	float x,y; //cartesian pos. of end effector
	
	std::cin >> x;
	std::cin >> y;
	float r_2 = x*x + y*y; //distance to end effector from origin
	
	float alpha= (r_2 - a1*a1 - a2*a2) / (2*a1*a2); // knee inner angle
	
	if (alpha > 1.0f) alpha = 1.0f;
	if (alpha < -1.0f) alpha = -1.0f;
	
	float phi2 = -acos(alpha); //knee up j2 angle
	
	float phi1 = atan2(y,x) - atan2(a2*sin(phi2), a1+a2*cos(phi2));
	
	std::cout << phi1 << "\n";
	std::cout << phi2;
	
	return 0;
	
}