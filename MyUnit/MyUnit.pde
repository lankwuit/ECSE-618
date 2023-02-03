/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  


/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeterh                     = 140;
float             pixelsPerMeterv                     = 80;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* end effector radius in meters */
float             rEEx                                 = 10/pixelsPerMeterh;
float             rEEy                                 = 10/pixelsPerMeterv;


/* virtual wall parameter  */
float             kWall                               = 450;
PVector           fWall                               = new PVector(0, 0);

//vertical walls declarations
PVector           pev1                                = new PVector(0, 0);
PVector           pev2                                = new PVector(0, 0);
PVector           pev3                                = new PVector(0, 0);
PVector           pev4                                = new PVector(0, 0);
PVector           pev5                                = new PVector(0, 0);
PVector           pev6                                = new PVector(0, 0);
PVector           pev7                                = new PVector(0, 0);
PVector           pev8                                = new PVector(0, 0);
PVector           pev9                                = new PVector(0, 0);
PVector           pev10                                = new PVector(0, 0);
PVector           pev11                                = new PVector(0, 0);

PVector           posv1                             = new PVector(0,0);
PVector           posv2                             = new PVector(1, 0);
PVector           posv3                             = new PVector(2, 0);
PVector           posv4                             = new PVector(3, 0);
PVector           posv5                             = new PVector(3, 0);
PVector           posv6                             = new PVector(4, 0);
PVector           posv7                             = new PVector(4, 0);
PVector           posv8                             = new PVector(5, 0);
PVector           posv9                             = new PVector(6, 0);
PVector           posv10                             = new PVector(7, 0);
PVector           posv11                             = new PVector(7, 0);

//horizontal wall declarations
PVector           peh1                                = new PVector(0, 0);
PVector           peh2                                = new PVector(0, 0);
PVector           peh3                                = new PVector(0, 0);
PVector           peh4                                = new PVector(0, 0);
PVector           peh5                                = new PVector(0, 0);
PVector           peh6                                = new PVector(0, 0);
PVector           peh7                                = new PVector(0, 0);
PVector           peh8                                = new PVector(0, 0);
PVector           peh9                                = new PVector(0, 0);
PVector           peh10                                = new PVector(0, 0);
PVector           peh11                                = new PVector(0, 0);
PVector           peh12                                = new PVector(0, 0);
PVector           peh13                                = new PVector(0, 0);

PVector           posh1                             = new PVector(0, 1);
PVector           posh2                             = new PVector(0, 1);
PVector           posh3                             = new PVector(0, 1);
PVector           posh4                             = new PVector(0, 2);
PVector           posh5                             = new PVector(0, 3);
PVector           posh6                             = new PVector(0, 3);
PVector           posh7                             = new PVector(0, 4);
PVector           posh8                             = new PVector(0, 4);
PVector           posh9                             = new PVector(0, 5);
PVector           posh10                             = new PVector(0, 6);
PVector           posh11                             = new PVector(0, 7);
PVector           posh12                             = new PVector(0, 8);
PVector           posh13                             = new PVector(0, 8);




/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference World size */
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;


/* graphical elements */
PShape endEffector;
PShape v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,h1,h2,h3,h4,h5,h6,h7,h8,h9,h10,h11,h12,h13;
/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 650);
  
  /* device setup -- not changing */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */ 
  haplyBoard          = new Board(this, "COM8", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
   
  
  /* visual elements setup */
  background(0);
  
  /* create pantagraph graphics// removed */
  create_ee();
  /* create wall graphics */
  //input wall locations based on the coordinate
  v1 = create_wall(posv1.x-rEE, 1, posv1.x+rEE, 5);
  v2 = create_wall(posv2.x-rEE, 2, posv2.x+rEE, 4);
  v3 = create_wall(posv3.x-rEE, 5, posv3.x+rEE, 8);
  v4 = create_wall(posv4.x-rEE, 1, posv4.x+rEE, 3);
  v5 = create_wall(posv5.x-rEE, 4, posv5.x+rEE, 6);
  v6 = create_wall(posv6.x-rEE, 1, posv6.x+rEE, 3);
  v7 = create_wall(posv7.x-rEE, 4, posv7.x+rEE, 6);
  v8 = create_wall(posv8.x-rEE, 1, posv8.x+rEE, 3);
  v9 = create_wall(posv9.x-rEE, 1, posv9.x+rEE, 2);
  v10 = create_wall(posv10.x-rEE, 1, posv10.x+rEE, 4);
  v11 = create_wall(posv11.x-rEE, 6, posv11.x+rEE, 7);
  
  h1 = create_wall(0, posh1.y+rEE, 3, posh1.y+rEE);
  h2 = create_wall(4, posh2.y+rEE, 5, posh2.y+rEE);
  h3 = create_wall(6, posh3.y+rEE, 7, posh3.y+rEE);
  h4 = create_wall(1, posh4.y+rEE, 2, posh4.y+rEE);
  h5 = create_wall(2, posh5.y+rEE, 3, posh5.y+rEE);
  h6 = create_wall(4, posh6.y+rEE, 6, posh6.y+rEE);
  h7 = create_wall(1, posh7.y+rEE, 3, posh7.y+rEE);
  h8 = create_wall(4, posh8.y+rEE, 7, posh8.y+rEE);
  h9 = create_wall(0, posh9.y+rEE, 2, posh9.y+rEE);
  h10 = create_wall(4, posh10.y+rEE, 7, posh10.y+rEE);
  h11 = create_wall(4, posh11.y+rEE, 7, posh11.y+rEE);
  h12 = create_wall(2, posh12.y+rEE, 3, posh12.y+rEE);
  h13 = create_wall(3, posh13.y+rEE, 4, posh13.y+rEE);
  
  v1.setStroke(color(0));
  v2.setStroke(color(0));
  v3.setStroke(color(0));
  v4.setStroke(color(0));
  v5.setStroke(color(0));
  v6.setStroke(color(0));  
  v7.setStroke(color(0));
  v8.setStroke(color(0));
  v9.setStroke(color(0));
  v10.setStroke(color(0));
  v11.setStroke(color(0));
  
  h1.setStroke(color(0));
  h2.setStroke(color(0));
  h3.setStroke(color(0));
  h4.setStroke(color(0));
  h5.setStroke(color(0));
  h6.setStroke(color(0));  
  h7.setStroke(color(0));
  h8.setStroke(color(0));
  h9.setStroke(color(0));
  h10.setStroke(color(0));
  h11.setStroke(color(0));
  h12.setStroke(color(0));
  h13.setStroke(color(0));
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255); 
    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(device_to_graphics(posEE)); 
      
      
      /* haptic wall force calculation */
      fWall.set(0, 0);
      
      penWall.set(0, (posWall.y - (posEE.y + rEE)));
      
      if(penWall.y < 0){
        fWall = fWall.add(penWall.mult(-kWall));  
      }
      
      fEE = (fWall.copy()).mult(-1);
      fEE.set(graphics_to_device(fEE));
      /* end haptic wall force calculation */
    }
    
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
//removed pantagraph
void create_ee(){
  endEffector = createShape(ELLIPSE,);
  endEffector.setFill(color(255,0,0));
}

PShape create_wall(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeterh * x1;
  y1 = pixelsPerMeterv * y1;
  x2 = pixelsPerMeterh * x2;
  y2 = pixelsPerMeterv * y2;
  
  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}


void update_animation(float th1, float th2, float xE, float yE){
  background(255);
  
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  th1 = 3.14 - th1;
  th2 = 3.14 - th2;

//removed for pantagraph and joint animations in sketch wall
  shape(h1);
  shape(h2);
  shape(h3);
  shape(h4);
  shape(h5);
  shape(h6);
  shape(h7);
  shape(h8);
  shape(h9);
  shape(h10);
  shape(h11);
  shape(h12);
  shape(h13);
  
  shape(v1);
  shape(v2);
  shape(v3);
  shape(v4);
  shape(v5);
  shape(v6);
  shape(v7);
  shape(v8);
  shape(v9);
  shape(v10);
  shape(v11);
  
  
  translate(xE, yE);
  shape(endEffector);
}


PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}



/* end helper functions section ****************************************************************************************/




 
