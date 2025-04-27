/**
   This file is part of BioSwarm Simulator.

    BioSwarm Simulator is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    BioSwarm Simulator is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <https://www.gnu.org/licenses/>
 **/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <GL/glew.h>
#include <GL/freeglut.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H

#include "common/shader_utils.h"
#include "GL/gl.h"
#include <map>
#include <vector>
#include <random>

#include <iostream>

using namespace std;
//-------------------------------
// Global variables
char title[] = "BioSwarm v42"; //Window title
int windowWidth  = 1200;     // Window width
int windowHeight = 768;     // Window height
int windowPosX   = 50;      // Window top-left corner x (where to display the window)
int windowPosY   = 50;      // Window top-left corner y

int REFRESH = 0;      // Refresh period in milliseconds

struct Character {
    unsigned int TextureID;  // ID handle of the glyph texture
    glm::ivec2   Size;       // Size of glyph
    glm::ivec2   Bearing;    // Offset from baseline to left/top of glyph
    unsigned int Advance;    // Offset to advance to next glyph
};

std::map<char, Character> Characters;

typedef struct {
  double x;
  double y;
} Position;

typedef struct {
  double x;
  double y;
} Velocity;

typedef struct {
  double x;
  double y;
} Pose;

typedef struct {
  Position position;
  Velocity velocity;
  Pose pose; //Heynolds: indicates if the current movement is still possible
  double fitness;
  Position bestPosition;
  double bestFitness;
  int color;
} Particle;

std::vector<Particle> particles; //dynamic array size
Position globalBestPosition;
double globalBestFitness;

double globalMeanFitness;
double globalStdFitness;

const double MIN_VALUE=-0.1; //random generator 
const double MAX_VALUE=0.1;  //random generator 

double MAX_LIMIT=0.8; //for point goal
double MIN_LIMIT=-0.8; //for point goal

const double LINE_MAX_LIMIT=0.8; //line grade
const double LINE_MIN_LIMIT=-0.8; //line grade

unsigned int ITERATIONS=0;
uint NUM_PARTICLES=0;
int COLLISION=0;
int COLOR=0;
int EVO=0;
int DEBUG=0;
int SHOW_AREA=0;
int SHOW_GRID=0;
//-------------------------------


GLuint program;
GLint attribute_coord;
GLint uniform_tex;
GLint uniform_color;

struct point {
	GLfloat x;
	GLfloat y;
	GLfloat s;
	GLfloat t;
};

GLuint vbo;

FT_Library ft;
FT_Face face;

const char *fontfilename;


  double getRandom(){
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(MIN_VALUE, MAX_VALUE);
    return dist(mt);
  }//end getRandom
  
  Position getRandomPosition(){
    Position p;
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(MIN_VALUE, MAX_VALUE);
    p.x = dist(mt) * 5; //*5 to give values far from origin 
    p.y = dist(mt) * 5;
    //if(DEBUG)
    //cout << "p.x:[" << p.x << "] p.y:[" << p.y << "]" << endl;
    return p;

  }//end getRandomPosition

  Velocity getRandomVelocity(){
    Velocity v;
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(MIN_VALUE, MAX_VALUE);
    v.x = dist(mt);
    v.y = dist(mt);
    return v;

  }//end getRandomVelocity



//Test functions
double TEST_F1=0;
double TEST_F2=0;
double TEST_F3=0;
double TEST_F4=0;
double FACTOR=0; //estimate to normalize 
//double TEST_F5=0;


//---Test functions
//Sum square function
void f1(){

  TEST_F1=0;
  
  for(uint i=0; i<NUM_PARTICLES; i++)
    TEST_F1 += i+pow(particles[i].fitness,2);  //FIXME: It should be the particles coordinates, and not its fitness.
                                               //       To fix this, plot a single point in the graph.
                                               //Put the surface in black-white and dots in red or black

  //Normalize values
  TEST_F1 = TEST_F1/FACTOR;
  
}

//Sphere Function
void f2(){

 TEST_F2=0;
  
  for(uint i=0; i<NUM_PARTICLES; i++)
    TEST_F2 += pow(particles[i].fitness,2);

  TEST_F2 /= FACTOR;
}

/*//From article
void f2(){

  TEST_F2=0;
  double a=0;
  double b=0;
  for(uint i=0; i<NUM_PARTICLES; i++)
    a += particles[i].fitness;

  for(uint i=0; i<NUM_PARTICLES; i++)
    b *= particles[i].fitness;

  TEST_F2 = a+b;
  }*/

//Sum of Different Powers Function
void f3(){

  TEST_F3=0;
  for(uint i=0; i<NUM_PARTICLES; i++)
    TEST_F3 += i*pow(particles[i].fitness,2);

  TEST_F3 /= FACTOR;
  
}

//Schwefel Function (many local minima)
void f4(){
  
  TEST_F4=0;
  for(uint i=0; i<NUM_PARTICLES; i++)
    TEST_F4 += particles[i].fitness*sin(sqrt(abs(particles[i].fitness)));

  TEST_F4 = 418.9829*NUM_PARTICLES - TEST_F4;

  TEST_F4 /= FACTOR;
  
  }

/*//Rotated Hyper-Ellipsoid function
void f4(){

  TEST_F4=0;
  for(uint i=0; i<NUM_PARTICLES; i++)
    for(uint j=0; j<i; j++)
      TEST_F4 += pow(particles[i].fitness,2);

  TEST_F4 /= FACTOR;
  }*/

void tests(){

  f1();
  f2();
  f3();
  f4();
  //f5();
}


//---

double calculateFitness(Particle particle){

    Position p = particle.position;
        
    //cout << "2Pass here" << endl;
  
    switch(EVO){
      //Case 0: updates Boids: not fitness here
      //Case 1: Boids up-right
      //Case 2: Boids
    case 3: {
      //Theorem 1) (Optimum) Specific on each corner
      //Theorem 1.1) (Optimum) up-rightside corner (Point most distant from (-0.5,-0,5) )
      return sqrt(pow(p.x-(-0.5),2)+pow(p.y-(-0.5),2));
      break;
    }
    case 4: {
      //(Variation) From this same idea, p.x and p.y produces the same result with minor computation.
      return +p.x + p.y;
      break;
    }
      //---
    case 5: {
      //Theorem 1.2) (Optimum) down-rightside corner (Point most distant from (-0.5,0.5) )
      return sqrt(pow(p.x-(-0.5),2)+pow(p.y-(0.5),2));
      break;
    }
    case 6: {
      //(Variation) From this same idea, +p.x and -p.y produces the same result with minor computation.
      return +p.x - p.y;
    }
      //---
    case 7: {
      //Theorem 1.3) (Optimum) Euclidian distance: down-leftside corner (Point most distant from (0.5,0.5) )
      return sqrt(pow(p.x-(0.5),2)+pow(p.y-(0.5),2));
      break;
    }
    case 8: {
      //From this same idea, -p.x and -p.y produces the same result with minor computation. Conclusion: PSO contributes to perform minor computation in fitness to reach the same result of Euclidian distance to reach the goal. Minor variations aims to reach many near-optimum results that are progressivelly combined to obtain an optimum result, as presented.
      return -p.x - p.y;
      break;
    }
      //---
    case 9: {
      //So, in PSO, fitness=EuclidianDistance(x,y) is equivalent to fitness=Delta(x,y).

      //Theorem 1.4) (Optimum) up-leftside corner (Point most distant from (0.5,-0.5) )
      return sqrt(pow(p.x-(0.5),2)+pow(p.y-(-0.5),2));
      break;
    }
    case 10: {
      //From this same idea, -p.x and p.y produces the same result with minor computation.
      return -p.x + p.y;
      break;
    }
      //---
    case 11: {
      //Theorem 2.1) Sphere function
      return p.x * p.x + p.y * p.y;   
      break;
    }
    case 12: {
      //Theorem 2.2) (Optimum) The more distant point from itself (or from the middle) is on a random corner
      //return sqrt(pow(p.x,2)+pow(p.y,2));

      //Rastrigin
      double v[2] = {p.x, p.y}; 
      int d = sizeof(v)/sizeof(double);
      double sum1 = 0;
    
      for(int ii=0;ii<d;ii++){
	sum1 = sum1 + (pow(v[ii],2) - 10*cos(2*M_PI*v[ii]));
      }

      return 10*d + sum1;

      
      break;
    }
    case 13: {
      //Theorem 2.3) (Optimum) Sum of distances from each corner combination.
      //return
      //  sqrt(pow(p.x-(0.5),2)+pow(p.y-(0.5),2))+
      //  sqrt(pow(p.x-(-0.5),2)+pow(p.y-(-0.5),2))+
      //  sqrt(pow(p.x-(-0.5),2)+pow(p.y-(0.5),2))+
      //  sqrt(pow(p.x-(0.5),2)+pow(p.y-(-0.5),2));
      //Ackley function
      //Commented bellow due value of convergence be near at -200. 
      //return (-200 * exp(-0.2*sqrt(p.x*p.x+p.y*p.y)));

      //http://www.sfu.ca/~ssurjano/ackley.html
      double xx[2] = {p.x, p.y}; //[x,y]
      int d = sizeof(xx)/sizeof(double);

      double c = 2*M_PI;
      
      double b = 0.2;

      double a = 20;
      double sum1 = 0;
      double sum2 = 0;
      double xi=0;
      for(int ii=0;ii<d;ii++){
	xi = xx[ii];
      
	sum1 = sum1 + pow(xi,2);
	sum2 = sum2 + cos(c*xi);
      }

      double term1 = -a * exp(-b*sqrt(sum1/d));
      double term2 = -exp(sum2/d);

      return term1 + term2 + a + exp(1);
      
      break;
    }
      //-----
    case 14: {
      //Theorem 3) (Optimum) Middle. Reach the point most far from corners.
      //
      //return -(p.x * p.x + p.y * p.y);  
      //Adjiman test function
      return cos(p.x)*sin(p.y)-(p.x/((p.y*p.y)+1));
      break;
    }

      
    case 15: {
      //But, how to converge in a specific point?
      //return -(sqrt(p.x * 0.35 + p.y * 0.15)); //Nop. Non-convergence. The same for positive formula.
      //Produces convergence on some corner
      //return p.x * p.x + p.y * p.y;
      //Specific in the middle. Reach the point most far from corners.
      //return -(p.x * p.x + p.y * p.y);
      //
      //Theorem 4) (Optimum) Supernova convergence behavior
      //Theorem 4.1) (Optimum) //*Up-right supernova with ternary evaluation of distances. Return x+y.
      //double r = getRandom(); //perturbation cannot be randomly. It must be fixed (any value bellow MAX_LIMIT and above MIN_LIMIT is allowed)
      //
      //                                                          (Larger values gives faster supernova behavior. Eg.: MAX_LIMIT/2 > MAX_LIMIT*0.1)
      //Theorem 4.1.1) Fast supernova
      //The two lines bellow also works
      //double x = p.x-(MAX_LIMIT/2)==0?1:p.x-(MAX_LIMIT/2); //Higher reduces MAX_LIMIT 
      //double y = (p.y-abs(MIN_LIMIT/2))==0?1:p.y-abs(MIN_LIMIT/2);
      double x = p.x-(MAX_LIMIT/2); //Higher reduces MAX_LIMIT
      double y = p.y-abs(MIN_LIMIT/2);
      
      return x+y;
      break;
    }
    case 16: {
      //Theorem 4.1.2) Slow supernova
      double r = 0.1; //Control of supernova is the interval between [0 (slow), MAX_LIMIT/2 (faster)] 
      double x = p.x-(MAX_LIMIT-r)==0?1:p.x-(MAX_LIMIT-r); 
      double y = p.y-(abs(MIN_LIMIT)-r)==0?1:p.y-(abs(MIN_LIMIT)-r);
      return x+y;
      break;
    }
    case 17: {
      //Theorem 4.2.1) (Optimum) //Fast supernova. Down-right, but no supernova behavior. Return x-y.
      double x = (p.x-MAX_LIMIT/2)==0?1:p.x-MAX_LIMIT/2;  // divide / 2 to give some perturbation (any value bellow MAX_LIMIT/2 and above MIN_LIMIT is allowed)
      double y = (p.y-MIN_LIMIT/2)==0?1:p.y-MIN_LIMIT/2;
      return x-y;
      break;
    }
      	
    case 18: {
	//Theorem 4.3) (Optimum) //Down-left supernova with ternary evaluation of distances. Return x+y;
	double x = (p.x-MAX_LIMIT/2)==0?1:-p.x+MAX_LIMIT/2;
	double y = (p.y-MIN_LIMIT/2)==0?1:-p.y+MIN_LIMIT/2;
	return x+y;
	break;
      }
    case 19: {
      //Theorem 4.4) (Optimum) //Up-left, but no supernova behavior. Return x-y.
      double x = (p.x-MAX_LIMIT/2)==0?1:-p.x+MAX_LIMIT/2;
      double y = (p.y-MIN_LIMIT/2)==0?1:-p.y+MIN_LIMIT/2;
      return x-y;
      break;
      }
      //Giving the limits in the command-line is sufficient to reach any point.
      //No further cases are necessary.

    case 20: {
      
      std::random_device rd;
      std::mt19937 mt(rd());
      std::uniform_real_distribution<double> dist(0, 360);
      double r1=dist(mt);

      std::random_device rd2;
      std::mt19937 mt2(rd2());
      std::uniform_real_distribution<double> dist2(0, 180);
      double r2=dist(mt2);

      double x1=0.61;
      double x2=0.61;
      return p.x*abs(sin(r1))+r2*sin(r2)*abs(x1*particle.bestFitness-x2*p.x);

      break;
    }

    
    
    default: {
      //Theorem 1.1) (Optimum) up-rightside corner (Point most distant from (-0.5,-0,5) )
      return sqrt(pow(p.x-(-0.5),2)+pow(p.y-(-0.5),2));
      break;
    }

    }    
    
  }//end calculateFitness

  Position getGlobalBestPosition(){
    return globalBestPosition;
  }

  double getGlobalBestFitness(){
    return globalBestFitness;
  }


bool collision(uint index, Position pos){

  bool result = false;
  bool found = false;
  double dist=0;
  for(uint i=0; i<particles.size() && !found; i++){
    if(index!=i){
      dist = sqrt(pow(particles[i].position.x-pos.x,2)+pow(particles[i].position.y-pos.y,2));
      //if(DEBUG)
      //	cout << "Dist: " << dist << endl;
      if(dist<pow(10,-2)){ //Collision
	result=true;
	found=true;
      }//end if
    }//end inf
  }//end for

  return result; 
}

void initPSO() {

  //if(DEBUG)
  // cout << "Log:1" << endl;

  globalBestFitness = 1000; //Initial large value

   Position p;
   //Initialize the population on random location
   p.x=0;
   p.y=0;
   
   globalBestPosition = p;

   if (DEBUG)
    cout << "iteration | Mean Fitness | Global bestFitness | Global bestPosX | Global bestPosY | Mean velX | Mean velY | Standard Deviation" << endl;

   globalMeanFitness=0;
   double averageVelX=0;
   double averageVelY=0;
   
   int color=1;
   Position pos;
   
    //Initialize position and velocity randomly
   for(uint i=0; i<NUM_PARTICLES; i++){

     Particle pa;
     particles.push_back(pa);

     //if(DEBUG)
     // cout << "Log:2" << endl;
      
     //if(DEBUG)
     //  cout << "Log:3" << endl;
  
     pos = getRandomPosition();
     
     /*To not change the initial population, keep it with collision
       while(collision(i,pos))
       pos = getRandomPosition();
     */
     
     particles[i].position = pos;
     particles[i].velocity = getRandomVelocity();
      //if(DEBUG)
      //	cout << "X:" << particles[i].position.x << " Y:" << particles[i].position.y << " VX:" << particles[i].velocity.x << " VY: " << particles[i].velocity.y << endl;
      
      //if(DEBUG)
      //	cout << "Log:4" << endl;
      
      //Initialize fitness and best position
     particles[i].fitness = calculateFitness(particles[i]); //here is already the test function.Eg.: (EVO11): z=x^2+y^2.
      //if(DEBUG)
      //	cout << "Log:5" << endl;
      
     particles[i].bestFitness = particles[i].fitness;  //The initial 'bestFitness' is its own fitness 
      //if(DEBUG)
      //	cout << "Log:6" << endl;
 
     particles[i].bestPosition = particles[i].position; //The initial 'bestPosition' is its own position
      //if(DEBUG)
      //	cout << "Log:7" << endl;
 
      //Update global best position and fitness
     if(particles[i].fitness < globalBestFitness){ //The initial 'globalBest' it is not important. This value will be updated latter.
	globalBestFitness = particles[i].fitness;
	globalBestPosition = particles[i].position;
      }
      
      globalMeanFitness += particles[i].fitness;
      averageVelX += particles[i].velocity.x;
      averageVelY += particles[i].velocity.y;

      //Setup particle color
      std::random_device rd;
      std::mt19937 mt(rd());
      std::uniform_real_distribution<double> dist(1, 6); //red,green,blue,white,yellow
      color = (int) dist(mt);
      particles[i].color = color;

      //
      Pose pose;
      pose.x=0; //0:keeps the current movement in X, 1: needs to change
      pose.y=0; //0:keeps the current movement in Y, 1: needs to change
      particles[i].pose=pose;
      
    }//end for

   globalMeanFitness /= particles.size();
   averageVelX /= particles.size();
   averageVelY /= particles.size();

   globalStdFitness=0;
   for(uint i=0; i<NUM_PARTICLES; i++)
     globalStdFitness += pow(particles[i].fitness - globalMeanFitness, 2); 


   //Perform test functions to compare the obtained fitness. Here because each case has 'return'.
   tests();
   
   const string sep = " | ";
   if(DEBUG)
     cout << ITERATIONS << sep << globalMeanFitness << sep << globalBestFitness << sep << globalBestPosition.x << sep << globalBestPosition.y << sep << averageVelX << sep << averageVelY << sep << globalStdFitness << endl; 
   
}//end initPSO



void updateBoids1(){


  //if(DEBUG)
  //  cout << "Iterations:" << ITERATIONS << endl;
  ITERATIONS++;

  //cout << particles.size() << endl;
  
    //Update particle positions and velocities
  //double r1;
  //double r2;
  double vx;
  double vy;
  globalMeanFitness=0;
  double averageVelX=0;
  double averageVelY=0;
  
  Position pos;
  for(unsigned int i=0; i<particles.size(); i++){

    particles[i].velocity.x=0.1;
    particles[i].velocity.y=0.1;
    
    //   r1 = getRandom();
    //r2 = getRandom();
      
    //vx =
    //	0.5 * particles[i].velocity.x +
    //	2.0 * r1 * (particles[i].bestPosition.x - particles[i].position.x) +
    //	2.0 * r2 * (globalBestPosition.x - particles[i].position.x);

       double step=0.01;
       
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0, 2); 
    
    //int p = floor(dist(mt)); //takes random values between 0 and 1
    //cout << p << endl;
    if(particles[i].pose.x)
      vx = particles[i].position.x + step;
    else
      vx = particles[i].position.x - step;
    
      //Simulates the new position
      pos.x = vx;
      pos.y = particles[i].position.y;

      
      if( vx < MAX_LIMIT &&
	  vx > MIN_LIMIT){
	if(COLLISION){//activated flag
	  if(!collision(i,pos)){ //Evaluates the collision on new position
	    particles[i].position.x = vx;
	    //--Try to Alignment: average of local pose
	    pos.x=0;
	    for(uint k=0;k<NUM_PARTICLES;k++) //cannot reduce because particles can be stuck  
	      //if(i!=k && (particles[i].color==particles[k].color))
		pos.x += particles[i].position.x;
	    
	    pos.x /= NUM_PARTICLES;
	    if(!collision(i,pos)) //Evaluates the collision on new position
	      particles[i].position.x = pos.x;
	    else {
	      //--Try to Evacuate (my new approach): If everyone is going to hole, try a new pose
	      particles[i].pose.x=(particles[i].pose.x==0)?1:0; //Binary Complement 
	    }
	  } else {
	    //Collision: change pose
	    particles[i].pose.x=(particles[i].pose.x==0)?1:0; //Binary Complement
	    //cout << "ChangeX" << endl;
	  }
	} else { //No collision detection
	  particles[i].position.x = vx;
	}
      } else {
	//Collision in corner: change pose
	particles[i].pose.x=(particles[i].pose.x==0)?1:0; //Binary Complement
      }
      
      //r1 = getRandom();
      //r2 = getRandom();
      
      //vy =
      //0.5 * particles[i].velocity.y +
      //2.0 * r1 * (particles[i].bestPosition.y - particles[i].position.y) +
      //2.0 * r2 * (globalBestPosition.y - particles[i].position.y);

      if(particles[i].pose.y)
	vy = particles[i].position.y + step;
	else
	vy = particles[i].position.y - step;
	
      
      //Simulates the new position
      pos.x = particles[i].position.x;
      pos.y = vy;
      
      if( vy < MAX_LIMIT &&
	  vy > MIN_LIMIT){
	if(COLLISION){
	  if(!collision(i,pos)){ //--Try to Separate: Evaluates the collision on new position
	      particles[i].position.y = vy;
	    //--Try to Alignment: average of local pose
	    pos.y=0;
	    for(uint k=0;k<NUM_PARTICLES;k++)
	      //if(i!=k && (particles[i].color==particles[k].color))
		pos.y += particles[i].position.y;
	    
	    pos.y /= NUM_PARTICLES;
	    if(!collision(i,pos)) //Evaluates the collision on new position
	      particles[i].position.y = pos.y;
	    else {
	      //--Try to Evacuate (my new approach): If everyone is going to hole, try a new pose
	      particles[i].pose.y=(particles[i].pose.y==0)?1:0; //Binary Complement 
	    }
	      
	  } else {
	    //Collision: change pose
	    particles[i].pose.y=(particles[i].pose.x==0)?1:0; //Binary Complement
	    //cout << "ChangeY" << endl;
	  }
	  
	} else {
	  particles[i].position.y = vy;
	}
      } else {
	//Collision: change pose
	particles[i].pose.y=(particles[i].pose.x==0)?1:0; //Binary Complement
      }
      
      //Updates local fitness and local best position
      particles[i].fitness = calculateFitness(particles[i]);
      if(particles[i].fitness > particles[i].bestFitness){ //If my fitness > global best 
	particles[i].bestFitness = particles[i].fitness;
	particles[i].bestPosition = particles[i].position;
      }//end if

      //Update global best position and fitness
      if(particles[i].fitness > globalBestFitness){
	globalBestFitness = particles[i].fitness;
	globalBestPosition = particles[i].position;

      }//end if

      
      //if(DEBUG)
      //cout << ITERATIONS << sep << i << sep << particles[i].fitness << sep << particles[i].bestFitness << sep << "(" << particles[i].position.x << "," << particles[i].position.y << ")" << sep << "(" << particles[i].bestPosition.x << "," << particles[i].bestPosition.y << ")" << sep << particles[i].velocity.x << sep << particles[i].velocity.y << endl;
      globalMeanFitness += particles[i].fitness;
      averageVelX += particles[i].velocity.x;
      averageVelY += particles[i].velocity.y;
      
    }//end for

  globalMeanFitness /= particles.size();
  averageVelX /= particles.size();
  averageVelY /= particles.size();

  globalStdFitness=0;
  for(uint i=0; i<NUM_PARTICLES; i++)
    globalStdFitness += pow(particles[i].fitness - globalMeanFitness, 2); 


  //Perform test functions to compare the obtained fitness. Here because each case has 'return'.
  tests();
  
  const string sep = " | ";
  if(DEBUG)
    cout << ITERATIONS << sep << globalMeanFitness << sep << globalBestFitness << sep << globalBestPosition.x << sep << globalBestPosition.y << sep << averageVelX << sep << averageVelY << sep << sep << globalStdFitness << endl; 
  
}//end updateBoids1

void updateBoids2(){


  //if(DEBUG)
  //  cout << "Iterations:" << ITERATIONS << endl;
  ITERATIONS++;

  //cout << particles.size() << endl;
  
    //Update particle positions and velocities
  //double r1;
  //double r2;
  double vx;
  double vy;
  globalMeanFitness=0;
  double averageVelX=0;
  double averageVelY=0;
  
  Position pos;
  for(unsigned int i=0; i<particles.size(); i++){

    particles[i].velocity.x=0.1;
    particles[i].velocity.y=0.1;
    
    //   r1 = getRandom();
    //r2 = getRandom();
      
    //vx =
    //	0.5 * particles[i].velocity.x +
    //	2.0 * r1 * (particles[i].bestPosition.x - particles[i].position.x) +
    //	2.0 * r2 * (globalBestPosition.x - particles[i].position.x);

       double step=0.01;
       
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0, 2); 
    
    //int p = floor(dist(mt)); //takes random values between 0 and 1
    //cout << p << endl;
    if(particles[i].pose.x)
      vx = particles[i].position.x + step;
    else
      vx = particles[i].position.x - step;
    
      //Simulates the new position
      pos.x = vx;
      pos.y = particles[i].position.y;

      
      if( vx < MAX_LIMIT &&
	  vx > MIN_LIMIT){
	if(COLLISION){//activated flag
	  if(!collision(i,pos)){ //Evaluates the collision on new position
	    particles[i].position.x = vx;
	    //--Try to Alignment: average of local pose
	    pos.x=0;
	    for(uint k=0;k<NUM_PARTICLES;k++)  
	      pos.x += particles[i].position.x;
	    
	    pos.x /= NUM_PARTICLES;
	    if(!collision(i,pos)) //Evaluates the collision on new position
	      particles[i].position.x = pos.x;
	    
	  } else {
	    //Collision: change pose
	    particles[i].pose.x=1;
	  }
	} else { //No collision detection
	  particles[i].position.x = vx;
	}
      } else {
	//Collision in corner: change pose
	particles[i].pose.x=1;
      }
      
      //r1 = getRandom();
      //r2 = getRandom();
      
      //vy =
      //0.5 * particles[i].velocity.y +
      //2.0 * r1 * (particles[i].bestPosition.y - particles[i].position.y) +
      //2.0 * r2 * (globalBestPosition.y - particles[i].position.y);

      if(particles[i].pose.y)
	vy = particles[i].position.y + step;
	else
	vy = particles[i].position.y - step;
	
      
      //Simulates the new position
      pos.x = particles[i].position.x;
      pos.y = vy;
      
      if( vy < MAX_LIMIT &&
	  vy > MIN_LIMIT){
	if(COLLISION){
	  if(!collision(i,pos)){ //--Try to Separate: Evaluates the collision on new position
	      particles[i].position.y = vy;
	    //--Try to Alignment: average of local pose
	    pos.y=0;
	    for(uint k=0;k<NUM_PARTICLES;k++)  
	      pos.y += particles[i].position.y;
	    
	    pos.y /= NUM_PARTICLES;
	    if(!collision(i,pos)) //Evaluates the collision on new position
	      particles[i].position.y = pos.y;
	  } else {
	    //Collision: change pose
	    particles[i].pose.y=1;
	  }
	  
	} else {
	  particles[i].position.y = vy;
	}
      } else {
	//Collision: change pose
	particles[i].pose.y=1;
      }
      
      //Updates local fitness and local best position
      particles[i].fitness = calculateFitness(particles[i]);
      if(particles[i].fitness > particles[i].bestFitness){ //If my fitness > global best 
	particles[i].bestFitness = particles[i].fitness;
	particles[i].bestPosition = particles[i].position;
      }//end if

      //Update global best position and fitness
      if(particles[i].fitness > globalBestFitness){
	globalBestFitness = particles[i].fitness;
	globalBestPosition = particles[i].position;

      }//end if

      
      //if(DEBUG)
      //cout << ITERATIONS << sep << i << sep << particles[i].fitness << sep << particles[i].bestFitness << sep << "(" << particles[i].position.x << "," << particles[i].position.y << ")" << sep << "(" << particles[i].bestPosition.x << "," << particles[i].bestPosition.y << ")" << sep << particles[i].velocity.x << sep << particles[i].velocity.y << endl;
      globalMeanFitness += particles[i].fitness;
      averageVelX += particles[i].velocity.x;
      averageVelY += particles[i].velocity.y;
      
    }//end for

  globalMeanFitness /= particles.size();
  averageVelX /= particles.size();
  averageVelY /= particles.size();

  globalStdFitness=0;
  for(uint i=0; i<NUM_PARTICLES; i++)
    globalStdFitness += pow(particles[i].fitness - globalMeanFitness, 2); 


  //Perform test functions to compare the obtained fitness. Here because each case has 'return'.
  tests();
    
  const string sep = " | ";
  if(DEBUG)
    cout << ITERATIONS << sep << globalMeanFitness << sep << globalBestFitness << sep << globalBestPosition.x << sep << globalBestPosition.y << sep << averageVelX << sep << averageVelY << sep << sep << globalStdFitness << endl; 

}

void updatePSO(){


  //if(DEBUG)
  //  cout << "Iterations:" << ITERATIONS << endl;
  ITERATIONS++;

  //cout << particles.size() << endl;
  
    //Update particle positions and velocities
  double r1;
  double r2;
  double vx;
  double vy;
  globalMeanFitness=0;
  double averageVelX=0;
  double averageVelY=0;

  Position pos;
  for(unsigned int i=0; i<particles.size(); i++){
    
      r1 = getRandom();
      r2 = getRandom();

      vx =
    	0.5 * particles[i].velocity.x +
    	2.0 * r1 * (particles[i].bestPosition.x - particles[i].position.x) +
    	2.0 * r2 * (globalBestPosition.x - particles[i].position.x);
      
      /*vx =
	0.5*(1+r1) * particles[i].velocity.x +
	0.05 * r1 * (particles[i].bestPosition.x - particles[i].position.x) +
	0.05 * r2 * (globalBestPosition.x - particles[i].position.x);
      */
      
      particles[i].velocity.x = vx;

      //Simulates the new position
      pos.x = particles[i].position.x + vx;
      pos.y = particles[i].position.y;

      
      if((particles[i].position.x+particles[i].velocity.x) < MAX_LIMIT &&
	 (particles[i].position.x+particles[i].velocity.x) > MIN_LIMIT){
	if(COLLISION){
	  if(!collision(i,pos)) //Evaluates the collision on new position
	    particles[i].position.x += particles[i].velocity.x;
	} else { //No collision detection
	  particles[i].position.x += particles[i].velocity.x;
	}
      }
      
      r1 = getRandom();
      r2 = getRandom();

      vy =
	0.5 * particles[i].velocity.y +
	2.0 * r1 * (particles[i].bestPosition.y - particles[i].position.y) +
	2.0 * r2 * (globalBestPosition.y - particles[i].position.y);
      
      
      /*vy =
	0.5*(1+r1) * particles[i].velocity.y +
	0.05 * r1 * (particles[i].bestPosition.y - particles[i].position.y) +
	0.05 * r2 * (globalBestPosition.y - particles[i].position.y);
      */
    
      particles[i].velocity.y = vy;
      
      //Simulates the new position
      pos.x = particles[i].position.x;
      pos.y = particles[i].position.y + vy;
      
      if((particles[i].position.y+particles[i].velocity.y) < MAX_LIMIT &&
	 (particles[i].position.y+particles[i].velocity.y) > MIN_LIMIT){
	if(COLLISION){
	  if(!collision(i,pos)) //Evaluates the collision on new position)
	    particles[i].position.y += particles[i].velocity.y;
	} else {
	  particles[i].position.y += particles[i].velocity.y;
	}
      }
      
      //Updates local fitness and local best position
      //particles[i].fitness = calculateFitness(particles[i].position);
      particles[i].fitness = calculateFitness(particles[i]);

      //But, applying the 'test function' z=x^2+y^2, the particles not convergences in minimum global
      //The 'bestFitness' is from who is near from the known 'minimum global=0'

      if(particles[i].fitness < particles[i].bestFitness){ //If my fitness > personnel best 
      //if(sqrt(pow(particles[i].position.x,2)+pow(particles[i].position.y,2)) < particles[i].fitness){ //If my new movement < fitness:z=x^2+y^2
	particles[i].bestFitness = particles[i].fitness;
	particles[i].bestPosition = particles[i].position;
      }//end if

      //Update global best position and fitness
      if(particles[i].fitness < globalBestFitness){
	globalBestFitness = particles[i].fitness;
	globalBestPosition = particles[i].position;

      }//end if

      
      //if(DEBUG)
      //cout << ITERATIONS << sep << i << sep << particles[i].fitness << sep << particles[i].bestFitness << sep << "(" << particles[i].position.x << "," << particles[i].position.y << ")" << sep << "(" << particles[i].bestPosition.x << "," << particles[i].bestPosition.y << ")" << sep << particles[i].velocity.x << sep << particles[i].velocity.y << endl;
      globalMeanFitness += particles[i].fitness;
      averageVelX += particles[i].velocity.x;
      averageVelY += particles[i].velocity.y;
      
    }//end for

  globalMeanFitness /= particles.size();
  averageVelX /= particles.size();
  averageVelY /= particles.size();

  globalStdFitness=0;
  for(uint i=0; i<NUM_PARTICLES; i++)
    globalStdFitness += pow(particles[i].fitness - globalMeanFitness, 2); 
   
  //Perform test functions to compare the obtained fitness. Here because each case has 'return'.
  //tests();
    
  const string sep = " | ";
  if(DEBUG)
    cout << ITERATIONS << sep << globalMeanFitness << sep << globalBestFitness << sep << globalBestPosition.x << sep << globalBestPosition.y << sep << averageVelX << sep << averageVelY << sep << globalStdFitness << endl; 

}//end updatePSO





//---TEXT
int init_resources() {
	/* Initialize the FreeType2 library */
	if (FT_Init_FreeType(&ft)) {
		fprintf(stderr, "Could not init freetype library\n");
		return 0;
	}

	/* Load a font */
	if (FT_New_Face(ft, fontfilename, 0, &face)) {
		fprintf(stderr, "Could not open font %s\n", fontfilename);
		return 0;
	}

	program = create_program("text.v.glsl", "text.f.glsl");
	if(program == 0)
		return 0;

	attribute_coord = get_attrib(program, "coord");
	uniform_tex = get_uniform(program, "tex");
	uniform_color = get_uniform(program, "color");

	if(attribute_coord == -1 || uniform_tex == -1 || uniform_color == -1)
		return 0;

	// Create the vertex buffer object
	glGenBuffers(1, &vbo);

	return 1;
}

/**
 * Render text using the currently loaded font and currently set font size.
 * Rendering starts at coordinates (x, y), z is always 0.
 * The pixel coordinates that the FreeType2 library uses are scaled by (sx, sy).
 */
void render_text(const char *text, float x, float y, float sx, float sy) {
	const char *p;
	FT_GlyphSlot g = face->glyph;

	/* Create a texture that will be used to hold one "glyph" */
	GLuint tex;

	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glUniform1i(uniform_tex, 0);

	/* We require 1 byte alignment when uploading texture data */
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	/* Clamping to edges is important to prevent artifacts when scaling */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	/* Linear filtering usually looks best for text */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	/* Set up the VBO for our vertex data */
	glEnableVertexAttribArray(attribute_coord);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glVertexAttribPointer(attribute_coord, 4, GL_FLOAT, GL_FALSE, 0, 0);

	/* Loop through all characters */
	for (p = text; *p; p++) {
		/* Try to load and render the character */
		if (FT_Load_Char(face, *p, FT_LOAD_RENDER))
			continue;

		/* Upload the "bitmap", which contains an 8-bit grayscale image, as an alpha texture */
		glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, g->bitmap.width, g->bitmap.rows, 0, GL_ALPHA, GL_UNSIGNED_BYTE, g->bitmap.buffer);

		/* Calculate the vertex and texture coordinates */
		float x2 = x + g->bitmap_left * sx;
		float y2 = -y - g->bitmap_top * sy;
		float w = g->bitmap.width * sx;
		float h = g->bitmap.rows * sy;

		point box[4] = {
			{x2, -y2, 0, 0},
			{x2 + w, -y2, 1, 0},
			{x2, -y2 - h, 0, 1},
			{x2 + w, -y2 - h, 1, 1},
		};

		/* Draw the character on the screen */
		glBufferData(GL_ARRAY_BUFFER, sizeof box, box, GL_DYNAMIC_DRAW);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

		/* Advance the cursor to the start of the next character */
		x += (g->advance.x >> 6) * sx;
		y += (g->advance.y >> 6) * sy;
	}

	glDisableVertexAttribArray(attribute_coord);
	glDeleteTextures(1, &tex);
}

void display() {
  float sx = 2.0 / glutGet(GLUT_WINDOW_WIDTH);
  float sy = 2.0 / glutGet(GLUT_WINDOW_HEIGHT);

	glUseProgram(program);


	switch(COLOR){
	case 0: {
	  glClearColor(0, 0, 0, 1); //Black background
	  break;
	}
	case 1: {
	  glClearColor(1, 1, 1, 1); //White background
	  break;
	}
	default : {
	  glClearColor(0, 0, 0, 1); //Black background
	}
	}
	
	glClear(GL_COLOR_BUFFER_BIT);

	
	/* Enable blending, necessary for our alpha texture */
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	GLfloat black[4] = { 0, 0, 0, 1 };
	GLfloat white[4] = { 1, 1, 1, 1 };
	GLfloat red[4] = { 1, 0, 0, 1 };
	GLfloat green[4] = { 0, 1, 0, 1 };
	GLfloat blue[4] = { 0, 0, 1, 1 };
	GLfloat yellow[4] = { 1, 1, 0, 1 };
 
	/* Set font size to 48 pixels, color to black */
	FT_Set_Pixel_Sizes(face, 0, 48);

	//Numbering of axis
	switch(COLOR){
	case 0: {
	  glUniform4fv(uniform_color, 1, white);
	  break;
	}
	case 1: {
	  glUniform4fv(uniform_color, 1, black);
	  break;
	}
	default : {
	  glUniform4fv(uniform_color, 1, white);
	}
	}

	//All axis in red
	//glUniform4fv(uniform_color, 1, red);
	
	

	string label = "Particles: [";
	label.append(to_string(NUM_PARTICLES));
	label.append("] Strategy: [");
	label.append(to_string(EVO));
	label.append("] Search: [");
	label.append(to_string(ceil(MAX_LIMIT*100)/100).substr(0,4));
	label.append(",");
	label.append(to_string(ceil(MIN_LIMIT*100)/100).substr(0,5)); //5: to include negative signal
	label.append("] Iter.: [");
	label.append(to_string(ITERATIONS));
	label.append("] Best Fit: [");
	label.append(to_string(globalBestFitness));
	//label.append(to_string(ceil(globalBestFitness*100)/100).substr(0,4));
	label.append("] Mean: [");
	label.append(to_string(globalMeanFitness));
	//label.append(to_string(ceil(globalMeanFitness*100)/100).substr(0,4));
	label.append("] Std: [");
	label.append(to_string(globalStdFitness));
	//label.append(to_string(ceil(globalStdFitness*100)/100).substr(0,4));
	label.append("]");
	/*label.append("] F1: [");
	label.append(to_string(TEST_F1));
	label.append("] F2: [");
	label.append(to_string(TEST_F2));
	label.append("] F3: [");
	label.append(to_string(TEST_F3));
	label.append("] F4: [");
	label.append(to_string(TEST_F4));
	label.append("]");
	*/
	
	const char* sb = label.c_str();
	render_text(sb, LINE_MIN_LIMIT+0.05, -LINE_MAX_LIMIT+1.65, sx * 0.35, sy * 0.35);	

	
	
	render_text("Y", LINE_MIN_LIMIT-0.01, -LINE_MAX_LIMIT+1.65, sx * 0.25, sy * 0.25); //posX, posY, fontSizeX, fontSizeY (scaling texture)

	render_text("X", LINE_MIN_LIMIT+1.6, -LINE_MAX_LIMIT-0.1, sx * 0.25, sy * 0.25); //posX, posY, fontSizeX, fontSizeY (scaling texture)


	//Y axis
	double d=LINE_MIN_LIMIT;
	string s;
	double tick=0.1;
	double p=0;
	double q=0;
	for(;d<LINE_MAX_LIMIT;){
	  s = to_string(d);

	  p=(d==0)?1:0; //substring to not include negative signal in 0 (zero)
	  q=(d>0)?3:4; //symbols to taken from number (3 for positive numbers, 4 for negative numbers)
	   
	  const char* sa = s.substr(p,q).c_str();
	  
	  render_text(sa, LINE_MIN_LIMIT-0.055, d, sx * 0.25, sy * 0.25);
	  d+=tick;
	}

	//X axis
	d=LINE_MIN_LIMIT;
	for(;d<LINE_MAX_LIMIT;){
	  s = to_string(d);

	  p=(d==0)?1:0; //substring to not include negative signal in 0 (zero)
	  q=(d>0)?3:4; //symbols to taken from number (3 for positive numbers, 4 for negative numbers)
	   
	  const char* sa = s.substr(p,q).c_str();
	  
	  render_text(sa, d, LINE_MIN_LIMIT-0.045, sx * 0.25, sy * 0.25);
	  d+=tick;
	}
	
	//----------------------------------

	if(SHOW_GRID){
	  //Main Vertical line
	  glBegin(GL_LINES);
	  glVertex2f(LINE_MIN_LIMIT,LINE_MIN_LIMIT);
	  glVertex2f(LINE_MIN_LIMIT,LINE_MAX_LIMIT);
	  glEnd();
	  
	  //Main horizontal line
	  glBegin(GL_LINES);
	  glVertex2f(LINE_MIN_LIMIT,LINE_MIN_LIMIT);
	  glVertex2f(LINE_MAX_LIMIT,LINE_MIN_LIMIT);
	  glEnd();
	  
	  //Intermediary horizontal dashed lines
	  double step=0.1;
	  double xstep=step;
	  glEnable(GL_LINE_STIPPLE);
	  glLineStipple(1,0x0F0F); //defines the pattern for the dashed line
	  glBegin(GL_LINES);
	  for(int i=0; i<2*LINE_MAX_LIMIT/step; i++){
	    
	    glVertex2f(LINE_MIN_LIMIT,LINE_MIN_LIMIT+xstep);
	    glVertex2f(LINE_MAX_LIMIT,LINE_MIN_LIMIT+xstep);
	    
	    xstep+=step;
	  }
	  //Intermediary vertical dashed lines
	  double ystep=step;
	  for(int i=0; i<2*LINE_MAX_LIMIT/step; i++){
	    
	    glVertex2f(LINE_MIN_LIMIT+ystep,LINE_MIN_LIMIT);
	    glVertex2f(LINE_MIN_LIMIT+ystep,LINE_MAX_LIMIT);
	    
	    ystep+=step;
	  }
	  glEnd();
	  glDisable(GL_LINE_STIPPLE);
	  
	}//end SHOW_GRID

	
  //--------------------
  for(unsigned int i=0; i<particles.size(); i++){

    //All points in red
    //glUniform4fv(uniform_color, 1, red);

    switch(COLOR){
    case 0: {
      glUniform4fv(uniform_color, 1, white);
      break;
    }
    case 1: {
      glUniform4fv(uniform_color, 1, black);
      break;
    }

    case 2: {
        //Multicolor
      switch(particles[i].color){
      case 1: {
	glUniform4fv(uniform_color, 1, red);
	break;
      }
      case 2: {
	glUniform4fv(uniform_color, 1, green);
	break;
      }
      case 3: {
	glUniform4fv(uniform_color, 1, blue);
	break;
      }
      case 4: {
	glUniform4fv(uniform_color, 1, white);
	break;
      }
      case 5: {
	glUniform4fv(uniform_color, 1, yellow);
	break;
      }
      default:{
	glUniform4fv(uniform_color, 1, white);
	break;
      }
      }//end switch

    }//end case2
      
    }//end switch

    glPointSize(5);
    glBegin(GL_POINTS);
    
    glVertex2f(particles[i].position.x,particles[i].position.y);
   
    //if(DEBUG)
    //  cout << "X: " << particles[i].position.x << " Y:" << particles[i].position.y << endl;
    glEnd();
	//--------------------
  
    if(SHOW_AREA){
      GLfloat alpha[4] = { 1, 1, 0, 0.01 };
      glUniform4fv(uniform_color, 1, alpha);
      glBegin(GL_QUADS);
      glVertex2f(MIN_LIMIT,MIN_LIMIT);
      glVertex2f(MAX_LIMIT,MIN_LIMIT);
      glVertex2f(MAX_LIMIT,MAX_LIMIT);
      glVertex2f(MIN_LIMIT,MAX_LIMIT);
      glEnd();
    }
    
  }
  glutSwapBuffers();  // Swap front and back buffers (of double buffered mode)    

  switch(EVO){
  case 0: {
    updateBoids1(); //Reynolds
    break;
  }
  case 1: {
    updateBoids2(); //Modified - static goal up-right
    break;
  }
  case 2: {
    updateBoids2();
    //updateBoids3(); //In-progress...
    break;
  }
  default:{
    updatePSO(); //All other cases
    break;
  }
  }
    

  //----------------------------------
}

void free_resources() {
	glDeleteProgram(program);
}

void Timer(int value) {
   glutPostRedisplay();    // Post a paint request to activate display()
   glutTimerFunc(REFRESH, Timer, 0); // subsequent timer call at milliseconds
}


int main(int argc, char *argv[]) {

  if(argc != 11){
    printf("\tUsage:\t\t./bioswarm <1-NUM_PARTICLES> <2-COLLISION> <3-EVO> <4-MAX_LIMIT> <5-MIN_LIMIT> <6-COLOR> <7-SHOW_AREA> <8-SHOW_GRID> <9-REFRESH_RATE> <10-DEBUG>\
\n\tExample:\t./bioswarm 1000  0  11  0  0  2  0  0  0  1\
\n\n\t<1-NUM_PARTICLES>\tNumber of particles\
\n\t\t\t\t[1-n): Number of particles\
\n\t<2-COLLISION>\tCollision detection\
\n\t\t\t\t0: Disabled\n\t\t\t\t1: Enabled\
\n\t<3-EVO>\t\tEvolutive Strategy\
\n\t\t\t\t0: Boids: Separation,Alignment,Cohesion\
\n\t\t\t\t1: Boids: Static goal up-right\
\n\t\t\t\t2: Boids: Static goal 2\
\n\
\n\t\t\t\t3: Theorem 1.1: up-right \
\n\t\t\t\t4: Theorem 1.1: up-right (minor variation)\
\n\
\n\t\t\t\t5: Theorem 1.2: down-right\
\n\t\t\t\t6: Theorem 1.2: down-right (minor variation)\
\n\
\n\t\t\t\t7: Theorem 1.3: down-left\
\n\t\t\t\t8: Theorem 1.3: down-left (minor variation)\
\n\
\n\t\t\t\t9: Theorem 1.4: up-left\
\n\t\t\t\t10: Theorem 1.4: up-left (minor variation)\
\n\
\n\t\t\t\t11: Test Function: Sphere Function (Minimum global: (0.0))\
\n\t\t\t\t12: Test Function: Rastrigin Function (Minimum global: (0.0))\
\n\t\t\t\t13: Test Function: Ackley Function (Minimum global: (0.0))\
\n\t\t\t\t14: Test Function: Adjiman Function (Minimum global: (-2.02181))\
\n\
\n\t\t\t\t15: Theorem 4.1.1: Fast supernova (up-right)\
\n\t\t\t\t16: Theorem 4.1.2: Slow supernova (up-right)\
\n\
\n\t\t\t\t17: Theorem 4.2.1: Fast supernova down-right\
\n\
\n\t\t\t\t18: Theorem 4.3: down-left\
\n\t\t\t\t19: Theorem 4.4: up-right\
\n\
\n\t\t\t\t20: Custom F1\
\n\
\n\t<4-MAX_LIMIT>\tMaximum reacheable X point\
\n\t\t\t\t[0,1]: (Default: 0.8)\
\n\t<5-MIN_LIMIT>\tMinimum reacheable Y point\
\n\t\t\t\t[-1,0]: (Default: -0.8)\
\n\t<6-COLOR>\t\tColor display\
\n\t\t\t\t0: Monochromatic0 (default)\n\t\t\t\t1: Monochromatic1\n\t\t\t\t2: Multicolor\
\n\t<7-SHOW_AREA>\tShow area of [MAX_LIMIT,MIN_LIMIT].\
\n\t\t\t\t0: Disabled\n\t\t\t\t1: Enabled\
\n\t<8-SHOW_GRID>\tShow horizontal and vertical grids\
\n\t\t\t\t0: Disabled\n\t\t\t\t1: Enabled\
\n\t<9-REFRESH_RATE>\tRefresh rate (ms)\
\n\t\t\t\t[0,n): (Default: 0)\
\n\t<10-DEBUG>\tDebug\
\n\t\t\t\t0: Disabled\n\t\t\t\t1: Enabled\
\n\n");
    exit(1);
  }
    
  NUM_PARTICLES=atoi(argv[1]);
  COLLISION = atoi(argv[2]);
  EVO = atoi(argv[3]);
  MAX_LIMIT = atof(argv[4]);
  MIN_LIMIT = atof(argv[5]);
  MAX_LIMIT=(MAX_LIMIT==0)?0.8:abs(MAX_LIMIT); //X always positive
  MIN_LIMIT=(MIN_LIMIT==0)?-0.8:-abs(MIN_LIMIT); //Y always negative 
  COLOR = atoi(argv[6]);
  SHOW_AREA = atoi(argv[7]);
  SHOW_GRID = atoi(argv[8]);
  REFRESH = atoi(argv[9]);
  DEBUG = atoi(argv[10]);

  FACTOR = NUM_PARTICLES*200; //estimative to normalize bellow best Fit of PSO
  
  initPSO();
  glutInit(&argc, argv);
  glutInitContextVersion(2,0);
  glutInitDisplayMode(GLUT_DOUBLE); // Enable double buffered mode	
  glutInitWindowSize(windowWidth, windowHeight);  // Initial window width and height
  glutInitWindowPosition(windowPosX, windowPosY); // Initial window top-left corner (x, y)
  glutCreateWindow(title);      // Create window with given title

  fontfilename = "/usr/share/fonts/truetype/freefont/FreeSans.ttf";
  
  GLenum glew_status = glewInit();
  
  if (GLEW_OK != glew_status) {
    fprintf(stderr, "Error: %s\n", glewGetErrorString(glew_status));
    return 1;
  }
  
  if (!GLEW_VERSION_2_0) {
    fprintf(stderr, "No support for OpenGL 2.0 found\n");
    return 1;
  }
  
  if (init_resources()) {
    glutDisplayFunc(display); //Here, invokes 'updatePSO'
    glutTimerFunc(REFRESH, Timer, 0); //refresh screen to change particle positions
    glutMainLoop();
  }
  
  free_resources();
  return 0;
}
