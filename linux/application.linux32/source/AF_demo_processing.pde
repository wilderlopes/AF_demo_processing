/*
Adaptive Filtering Demo for 'Processing 2.2.1' - System Identification Task - Version 01.11.2014

    Copyright (C) 2014 onwards  Wilder Lopes

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
It uses the drawing library gwoptics, which is licensed under 
the terms of the GNU Lesser General Public License version 2.1 (a copy of this license
is provided with the Library.)    

Author: Wilder Bezerra Lopes (wilderlopes@gmail.com), Doctoral Student, University of Sao Paulo, Brazil.
Original version: 06 Nov. 2014.

Description: This demo shows a system identification task in which 'n' least-mean-squares adaptive filters (LMS-AFs),
with different step sizes, try to estimate the plant coefficients. The 2D coordinates (x,y) of the point named 'Target' are
collected in the so-called optimum vector wo = [x, y]', which is the system to be estimated.   

Goal: To provide a visual understanding of the convergence behavior for different AFs.

This code enables the user to:
  - Set the step size for n AFs;
  - Control the amount of measurement noise in the desired output d;
  - Control the amount of random-walk noise in the system wo;
  - Visualize the mean-square-deviation (MSD) learning curves;
  - Visualize the AF adaptation on the 2D plane ((x,y) position of the dots);
  
Required Libraries:
  - Papaya (Statistics library for Processing): http://adilapapaya.com/papayastatistics/
  - gwoptics (tool for 2D and 3D drawing): http://www.gwoptics.org/processing/gwoptics_p5lib/
  
  You should download those libraries and unpack them in the "libraries" folder of your 'Processing'
  environment. For Windows, this folder is usually located at "\User\Documents\Processing". In Linux-based
  systems it should be in "home/sketchbook".  
 
For further understanding of AFs: 
  - Ali H. Sayed, Adaptive Filters: http://amzn.com/0470253886
  - Paulo S. R. Diniz, Adaptive Filtering: Algorithms and Practical Implementation: http://amzn.com/1461441056
  
TO DO / ISSUES: 
  - Improve the colors and interface design;
  - Feature to change the size of axis (zoom);
  - Turn some things automatic, e.g., if you create n AFs, it should automatically create n traces (RollingLine2DTrace)
    for the Graph2D.
*/
 
import papaya.*;          //Library to enable matrix and linear algebra calculations;

Target target;            //Declaring the target (system to be identified);
LMS lms1, lms2, lms3;     //Declaring the LMS filters;

//PGraphics pg;

int   M = 2;               //System order;
float[] wo = {800, 400};   //System to be identified;  
float[] uj = {0, 0};
float[] regressor = new float[M];    //Creating regressor
float var_v = 1e-6;
float var_q = 0;           //Nonstationarity level;
float text_y = 20;     
int   sizeScreenX = 1200;  //Size of the screen - X;
int   sizeScreenY = 700;   //Size of the screen - Y;
int   sizeGraphX = 350;    
int   sizeGraphY = 200;
float graphX;
PFont f;

// importing plotting library
  import org.gwoptics.graphics.*;
  import org.gwoptics.graphics.graph2D.*;
  import org.gwoptics.graphics.graph2D.Graph2D;
  import org.gwoptics.graphics.graph2D.traces.ILine2DEquation;
  import org.gwoptics.graphics.graph2D.traces.RollingLine2DTrace;

  RollingLine2DTrace r1_mse,r2_mse,r3_mse;   // Declaring the traces for the Graph2D - MSE
  Graph2D g_mse;                             // Declaring Graph2D - MSE
  RollingLine2DTrace r1_emse,r2_emse,r3_emse;// Declaring the traces for the Graph2D - EMSE
  Graph2D g_emse;                            // Declaring Graph2D - EMSE
  RollingLine2DTrace r1_msd,r2_msd,r3_msd;   // Declaring the traces for the Graph2D - MSD
  Graph2D g_msd;                             // Declaring Graph2D - MSD
 
 
// Setup of the Processing application
void setup() {
  size(sizeScreenX, sizeScreenY);
  if (frame != null) {
    frame.setResizable(true);
  }  
  
  graphX = sizeGraphX + 90; //This variable is used to prevent the target to appear on top of the 2Dgraph; 
    
  float[] regressor = new float[M];    //Creating regressor
    for (int j = 0; j < M; j = j+1) {
      regressor[j] = 0;
    } 
  
  // Constructor for the LMS class   
  // LMS(color (R,G,B), float[] wo, float[] uj, float tempvar_v, float Xpos, float Ypos, float radius, float mu) 
  lms1 = new LMS(color(200,0,0), wo, regressor, var_v, graphX + 50, 600, 10, 0.005);
  lms2 = new LMS(color(0,200,0), wo, regressor, var_v, graphX + 300, 300, 10, 0.01);
  lms3 = new LMS(color(0,0,200), wo, regressor, var_v, graphX + 550, 500, 10, 0.05);
   
  background(0); // Screen background
  stroke(255);   // Font color  
  f = loadFont("Monospaced.bold-16.vlw"); // Font for the text 
 
  // 2D Graph - MSE
  class eq1_mse implements ILine2DEquation{
    public double computePoint(double x,int pos) {
      return lms1.mse_log;
    }    
  }  
  class eq2_mse implements ILine2DEquation{
    public double computePoint(double x,int pos) {
      return lms2.mse_log;
    }    
  }  
  class eq3_mse implements ILine2DEquation{
    public double computePoint(double x,int pos) {
      return lms3.mse_log;
    }    
  }   
  
  r1_mse = new RollingLine2DTrace(new eq1_mse(),200,0.1f); // Creating trace
  r1_mse.setTraceColour(200,0,0);
  r1_mse.setLineWidth(3);
  
  r2_mse = new RollingLine2DTrace(new eq2_mse(),200,0.1f); // Creating trace
  r2_mse.setTraceColour(0,200,0);
  r2_mse.setLineWidth(3);
  
  r3_mse = new RollingLine2DTrace(new eq3_mse(),200,0.1f); // Creating trace
  r3_mse.setTraceColour(0,0,200);
  r3_mse.setLineWidth(3);
   
  g_mse = new Graph2D(this, sizeGraphX, sizeGraphY, false); // Creating graph
  g_mse.setAxisColour(255,255,255);
  g_mse.setFontColour(255,255,255);
  g_mse.setYAxisMax(50);
  g_mse.setYAxisMin(-100);
  g_mse.setYAxisTickSpacing(20);
  g_mse.setYAxisLabel("MSE");
  g_mse.setXAxisLabel(" ");
  //g_mse.setDrawTickLabels(false);
  g_mse.setXAxisMax(5f); 
  g_mse.addTrace(r1_mse);
  g_mse.addTrace(r2_mse);
  g_mse.addTrace(r3_mse);
  g_mse.position.y = 200;
  g_mse.position.x = 70;
  
  
  
  // 2D Graph - MSD  
  class eq1_msd implements ILine2DEquation{
    public double computePoint(double x,int pos) {
      return lms1.msd_log;
    }    
  }  
  class eq2_msd implements ILine2DEquation{
    public double computePoint(double x,int pos) {
      return lms2.msd_log;
    }    
  }  
  class eq3_msd implements ILine2DEquation{
    public double computePoint(double x,int pos) {
      return lms3.msd_log;
    }    
  }   
  
  r1_msd = new RollingLine2DTrace(new eq1_msd(),200,0.1f); // Creating trace
  r1_msd.setTraceColour(200,0,0);
  r1_msd.setLineWidth(3);
  
  r2_msd = new RollingLine2DTrace(new eq2_msd(),200,0.1f); // Creating trace
  r2_msd.setTraceColour(0,200,0);
  r2_msd.setLineWidth(3);
  
  r3_msd = new RollingLine2DTrace(new eq3_msd(),200,0.1f); // Creating trace
  r3_msd.setTraceColour(0,0,200);
  r3_msd.setLineWidth(3);
   
  g_msd = new Graph2D(this, sizeGraphX, sizeGraphY, false); // Creating graph
  g_msd.setAxisColour(255,255,255);
  g_msd.setFontColour(255,255,255);
  g_msd.setYAxisMax(50);
  g_msd.setYAxisMin(-100);
  g_msd.setYAxisTickSpacing(20);
  g_msd.setYAxisLabel("MSD");
  g_msd.setXAxisLabel("Time");
  g_msd.setXAxisMax(5f); 
  g_msd.addTrace(r1_msd);
  g_msd.addTrace(r2_msd);
  g_msd.addTrace(r3_msd);
  g_msd.position.y = 460;
  g_msd.position.x = 70;   
}

// Inside draw() is where the action happens
void draw() {
  
  background(0);  // Brackground color
  
  // Updates target each time iteration to enable visualizing a nonstationary plant
  target = new Target(color(255), wo[0], wo[1], 15);  
  target.display();
  // Adapts and displays LMS filters
  // Constructor for the LMS class 
    
  lms1.adapt();     
  lms1.display();  
  lms2.adapt();
  lms2.display();  
  lms3.adapt();
  lms3.display();
  
  g_mse.draw(); // Call MSE graph  
  g_msd.draw(); // Call MSD graph

  // Drawing lines to separate the 2D visualization from the learning curves
  stroke(255);
  line(graphX, 0, graphX, height);
  line(graphX+5, 0, graphX+5, height);
  
  // Lines for code debugging below:  
  //print("msd = " + lms1.e_sd + " || msd_log = " + lms1.msd_log + "\n");
  //print("uj = [" + uj[0] + " " + uj[1] + "] \n");
  //print(lms1.e_sq_log);
     
                     
 //print("var_v = [" + lms1.var_v + "] \n");
   
  
  wo[0] += sqrt(var_q)*randomGaussian();  //Adding noise to the plant's coefficients -- This is done to observe the tracking behavior of the AFs;
  wo[1] += sqrt(var_q)*randomGaussian();  //Adding noise to the plant's coefficients -- This is done to observe the tracking behavior of the AFs;
   
  // Data displayed in the graphs
  textFont(f,16);                     //Specify font to be used;
  fill(255);                          //Specify font color;
  text("Adaptive Filtering Demo - System Identification - Ver. 01.11.2014",graphX + 10,text_y - 5);
  text("model   = " + "(" + wo[0] + "," + wo[1] + ")",10,text_y);  //Display wo weights;
  text("var_v   = " + 10*(log(var_v)/log(10)) + "dB",10,text_y + 20);  //Display nonstationarity level;
  text("var_q   = " + 10*(log(var_q)/log(10)) + "dB",10,text_y + 40);  //Display nonstationarity level;
  fill(200, 0, 0);
  text("LMS 1 | mu = " + lms1.mu + " | (" + lms1.xpos + "," + lms1.ypos + ")",10,text_y + 60);  //Display squared error of LMS 1;
  fill(0, 200, 0);
  text("LMS 2 | mu = " + lms2.mu + " | (" + lms2.xpos + "," + lms2.ypos + ")",10,text_y + 80); //Display squared error of LMS 1;
  fill(0, 0, 200);
  text("LMS 3 | mu = " + lms3.mu + " | (" + lms3.xpos + "," + lms3.ypos + ")",10,text_y + 100); //Display squared error of LMS 1;
  
  fill(0, 255, 100);
  textFont(f,16);                     //Specify font to be used;
  text("UP/DOWN: change var_v, CTRL: abrupt nonstationarity",graphX + 10,text_y+15);
  text("LEFT/RIGHT: change var_q, SHIFT: restart",graphX + 10,text_y+30);

  textFont(f,14);                     //Specify font to be used;
  text("Source Code: github.com/wilderlopes/AF_demo_processing - wilderlopes@gmail.com",graphX + 10,height-5);
  
}

//=================== Custom classes ======================

class Target {      //For the system to be identified
  color c;
  int   i = 0;
  float xpos;
  float ypos;
  float radius;   
  
  Target(color tempc, float tempxpos, float tempypos, float tempradius) {
    c = tempc;             //circle color
    xpos = tempxpos;       //X coordinate
    ypos = tempypos;       //Y coordinate
    radius = tempradius;   //circle radius  
  }
  
  void display() {         //Display circle
    stroke(255);
    fill(c);
    
    //Providing rules in case the target circle reachs the screen borders
    if (xpos > width) {
      xpos = width - 10;
    } else if (xpos < 490) {
      xpos = 490 + 10;      
    }     
    
    if (ypos > height) {
      ypos = height - 10;      
    } else if (ypos < 0) {
      ypos = text_y + 15;      
    } 
    
    ellipseMode(RADIUS);
    ellipse(xpos, ypos, radius, radius);
  }
}


class LMS {             //For each LMS AF
  color c;
  int   i = 0;
  float xpos;
  float ypos;
  float radius;
  float mu;
  float u, v, e, d, y;
  float mse, mse_log;             //Squared error
  int M;
  float var_v_lms;
  float[] sys = {0,0};          // System to be identified
  float[] w  = {0, 0};
  float[] e_sq_array = {0};
  float[] e_sq_log_array = {0};
  float[] e_sd = {0, 0};
  float msd, msd_log; 
  
  
  LMS(color tempc, float[] tempwo, float[] tempuj, float tempvar_v, float tempxpos, float tempypos, float tempradius, float tempmu) {
    c = tempc;            //circle color
    sys = tempwo;         //system to be identified
    w[0] = tempxpos;      //X coordinate
    w[1] = tempypos;      //Y coordinate
    uj = tempuj;          //regressor
    var_v_lms = tempvar_v;    //measurement noise variance
    radius = tempradius;  //circle radius
    mu = tempmu;          //step size
    M = tempwo.length;    //system order
    d = 0;                //desired output
    y = 0;                //estimated (AF) output     
  }
  
  void display() {        //display circle
    stroke(0);
    fill(c);
    ellipseMode(RADIUS);
    ellipse(xpos, ypos, radius, radius);
  }  
    
  void addNoise() {       //Increases the variance of the measumerement noise;
    var_v_lms = var_v_lms*sqrt(10);    
  }  
  
  void subNoise() {       //Decreases the variance of the measumerement noise;
    var_v_lms = var_v_lms/sqrt(10);    
  } 
    
  void adapt() {          //adaptation loop
    d = 0;
    y = 0;   
    v = sqrt(var_v_lms)*randomGaussian();   //measurement noise
    
    for (int j = 0; j < M; j = j+1) {      //Desired ouput
      d += uj[j] * sys[j];
    }
      
    for (int j = 0; j < M; j = j+1) {      //Estimated output
      y += uj[j] * w[j];
    }
  
    e = d + v - y;                         //Error
  
    for (int j = 0; j < M; j = j+1) {      //LMS update rule
      w[j] += mu*uj[j]*e;
    }
    
    u = randomGaussian();               //New sample for regressor (drawn from a zero-mean Gaussian process, variance = 1) 
     for (int j = M-1; j > 0; j = j-1) { //Shift register structure  
      uj[j] = uj[j-1];                     
    }                                  
    uj[0] = u;  //Updating the regressor
    
    mse = pow(e,2);                       //Mean-Square Error (MSE)
    mse_log = 10*(log(mse)/log(10));      //log10(MSE)
   
    e_sd = Mat.sum(sys,Mat.multiply(w,-1)); //error vector (wo - w) 
    msd  = Mat.dotProduct(e_sd,e_sd);       //Mean-Square Deviation (MSD)
    msd_log = 10*(log(msd)/log(10));        //log10(MSD)
       
    xpos = w[0];
    ypos = w[1];
    
    //Providing rules in case the target circle reachs the screen borders
    if (xpos > width) {
      xpos = width - 10;
    } else if (xpos < 490) {
      xpos = 490 + 10;      
    }     
    
    if (ypos > height) {
      ypos = height - 10;      
    } else if (ypos < 0) {
      ypos = text_y + 15;      
    }    
  }  
}



void keyPressed() {
  if (key == CODED) {
    
    if (keyCode == SHIFT) {      //Restart AFs from a random psosition;
      background(0);
      var_v = 1e-6;
      var_q = 0;
      lms1 = new LMS(color(200,0,0), wo, regressor, var_v, random(graphX,width), random(text_y+15,height), 10, 0.005);
      lms2 = new LMS(color(0,200,0), wo, regressor, var_v, random(graphX,width), random(text_y+15,height), 10, 0.01);
      lms3 = new LMS(color(0,0,200), wo, regressor, var_v, random(graphX,width), random(text_y+15,height), 10, 0.05);      
    }
    if (keyCode == CONTROL) {    //Abrupt nonstationarity;
      background(0);
      wo[0] = random(graphX,width);   // X axis coordinate;
      wo[1] = random(text_y + 15,height);  // Y axis coordinate;      
    }
    
   if (keyCode == UP) {     
      var_v = var_v*sqrt(10);   
      lms1.addNoise();             //Increases the variance of the measumerement noise for LMS1;    
      lms2.addNoise();             //Increases the variance of the measumerement noise for LMS2; 
      lms3.addNoise();             //Increases the variance of the measumerement noise for LMS3;      
    } else if (keyCode == DOWN) {  
      var_v = var_v/sqrt(10); 
      lms1.subNoise();             //Decreases the variance of the measumerement noise for LMS1; 
      lms2.subNoise();             //Decreases the variance of the measumerement noise for LMS2;  
      lms3.subNoise();             //Decreases the variance of the measumerement noise for LMS3;       
      if (var_v < 0) {
        var_v -= 0; 
      }
    } 
        
    if (keyCode == RIGHT) {   
      if (var_q == 0) {
        var_q = 1e-10;
      }      
      var_q = var_q*sqrt(10);      //Increases the level of random-walk noise;          
    } else if (keyCode == LEFT) {     
      var_q = var_q/sqrt(10);      //Increases the level of random-walk noise;            
      if (var_q < 1e-10) {              
        var_q = 0;
      }
    }         
  }
}


