int increment = .01;

//Converts a given input of degrees and converts it to radians
double Deg2Rad(double angleDegrees) {
  //converts a angle in degress and converts it to radians
  double angleRadians = angleDegrees * M_PI / 180.0 ;
  
  return angleRadians;//returns the angle in radians
}

double Rad2Deg(double angleRadians) {
  //converts a angle given in radians and converts it into degrees
  double angleDegrees = (180.0 / M_PI) * angleRadians;
  
  return angleDegrees;//returns the angle in degrees
}

double Quadratic(double a,double b,double c,double plusOrMinus){
  // solves for the given root with the quadratic formula 
  double root = (-b + plusOrMinus * sqrt(pow(b,2) - 4.0*a * c  ))/(2.0 * a);
  
  return root; //returns either the positive or negative root
}

double LandingDistance(double d[], double v0, double theta){
  //The Landing distance function solves for the langing distance of a projectile when the initial 
  //velocity, d values, and theta is given
  double d1 = d[0];
  double d2 = d[1];
  double d3 = d[2];
  
  double thetaRad = Deg2Rad(theta);//converts theta to radians
  
  //initilize constants
  double g = 9.81; 
  
  //solves for thee x and y velocities
  double v0x = v0 * cos(thetaRad);
  double v0y = v0 * sin(thetaRad);
  
  //solves for the initial x and y positions
  double x0 = d2*cos(thetaRad) - d3*sin(thetaRad);
  double y0 = d1 + d2*sin(thetaRad) + d3*cos(thetaRad);
  
  //Quadratic formula setup
  double a = -.5 * g; //a for quad
  double b = v0y;     // b for quad
  double c = y0;      // c for quad
  
  double tLand = Quadratic(a,b,c,-1);//runs the quadratice to find the landing time
  
  double xLand = x0 + v0x*tLand;//solves for the distance of x
  
  return xLand;// returns the landing distance
}

double RangeAngle(double d[],double v0){
  double maxDist = 0.0; //creates a scalor for max distance
  double theta = 25.0; //creates a scalor for max distance will be incrementing by 0.01
  double rangeAngle = 0.0; //sets range angle at 0
  for(theta = 35; theta <= 75; theta = theta + increment){ //solves for the maximum landing distance and corresponding angle
    double xLand = LandingDistance(d,v0,theta);
    if (xLand > maxDist){
      maxDist = xLand; //if old max distance is smaller than xland gives max distance the new xland
      rangeAngle = theta; //gives the correponding theta with xLand
    }
    else{
      break;
    }
  }
  return rangeAngle;
}
//Launch Angle 
double LaunchAngle(double d[],double v0,double xTarget){
  
  double thetaDeg = RangeAngle(d,v0); //solves for max launch angle
  double xLand = LandingDistance(d, v0, thetaDeg); //solves for max distance angle
  while (xTarget <= xLand){ //solves for the best launch angle corresponding to the target
      thetaDeg += .1;
      xLand = LandingDistance(d, v0, thetaDeg);
  }
  return thetaDeg;
}

//launch angle revised
double LaunchAngle2(double d[],double v0,double xTarget){
  
  double thetaDeg = RangeAngle(d,v0);//solves for the maximum angle
  double xLand = 0.60;//sets xland at 0
  double launchAngle = 75.0;//sets launch angle a 85
  do{ //while it is in the loop solves for landing distance 
    xLand = LandingDistance(d, v0, launchAngle);
    if (xLand >= xTarget){//if the xland is larger than xtarget stops the loop
      break;
    }
    launchAngle = launchAngle - 0.1; //decrements the loop every time by 0.01
  } while(launchAngle >= thetaDeg);
  
  return launchAngle;//returns the angle for corresponding xtarget
}

double ServoAngle(double H[], double thetaLaunch, double thetaServoOffset, double thetaLaunchOffset) {
  //ServoAngle solves for the required servo angle in the fourbard linkage when h1-h4 and theta launch are given
  double theta2Deg = thetaLaunch - thetaLaunchOffset; //creates a value theta 2 
  double theta2Rad = Deg2Rad(theta2Deg);//converts theta2 from degrees to radians
  
  //sets h1-4 with the corresponding H vector value
  double h1 = H[0];
  double h2 = H[1];
  double h3 = H[2];
  double h4 = H[3];
  
  double k1 = h1/h2;//solves for k1
  double k2 = h1/h4;//solves for k2
  double k3 = (pow(h1,2)+pow(h2,2)-pow(h3,2)+pow(h4,2))/(2*h2*h4);//solves for k3
  
  //Quadratic fomrula
  double a = cos(theta2Rad)- k1 - k2*cos(theta2Rad) + k3;// a for quad 
  double b = -2*sin(theta2Rad);// b for quad
  double c = k1 - (k2 + 1)*cos(theta2Rad) + k3;// c for quad
  
  double root = Quadratic(a,b,c,-1);//runs the Quadratic formula and give the negative root
  
  double theta4Rad = 2 * atan(root); //Solves for theta 4 in radians
  
  double theta4Deg = Rad2Deg(theta4Rad);// converts theta 4 to degrees
  double thetaServo = theta4Deg + thetaServoOffset;// solves for required servo angle
  return thetaServo;
}

void TargetServoAngles(double d[],double v0,double H[],double servoAngleOffset,double launchAngleOffset,double xTarget[]){
  
  double launchAngles = 0.0;
  double servoAngles = 0.0;
  int i = 0;
  for (i = 0; i < 6 ; i++){
    launchAngles = LaunchAngle2(d,v0,xTarget[i]);
    servoAngles = ServoAngle(H,launchAngles,servoAngleOffset,launchAngleOffset);

    writeToServo[i] = int(round(servoAngles));
  }
  
}

