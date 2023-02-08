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
float             pixelsPerMeterEE                    = 5000.0;
float             pixelsPerMeterh                     = 143;
float             pixelsPerMeterv                     = 80;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* end effector radius in meters */
//end effector size in pixels
float             rEEpixel                             = 25;
float             rEE                                  = rEEpixel/pixelsPerMeterEE;
// for serving the 140 x 80 pixel size subspace
float             rEEx                                 = rEEpixel/pixelsPerMeterh;
float             rEEy                                 = rEEpixel/pixelsPerMeterv;


/* virtual wall parameter  */
float             kWall                               = 450;
PVector           fWall                               = new PVector(0, 0);
PVector           penWall                               = new PVector(0, 0);

//vertical walls declarations
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
PVector           posv12                             = new PVector(4, 0);

//horizontal wall declarations
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
//the new sub-coordinate posEE
PVector           posEEsc                             = new PVector(0, 0);
PVector           velEE                               = new PVector(0, 0);

PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference World size */
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;


/* graphical elements */
PShape endEffector;
PShape v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,h1,h2,h3,h4,h5,h6,h7,h8,h9,h10,h11,h12,h13;
PShape triggertop, triggerbot, faketop, fakebot,finish;
PVector posftop = new PVector(3,0);
PVector posfbot = new PVector(0,6);

//for controlling if the trigger spot have ever been reached
boolean flagtop=false, flagbot=false;
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
  haplyBoard          = new Board(this, "COM9", 0);
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
  v1 = create_wall(posv1.x , 1, posv1.x , 5);
  v2 = create_wall(posv2.x , 2, posv2.x , 4);
  v3 = create_wall(posv3.x , 5, posv3.x , 8);
  v4 = create_wall(posv4.x , 1, posv4.x , 3);
  v5 = create_wall(posv5.x , 4, posv5.x , 8);
  v6 = create_wall(posv6.x , 1, posv6.x , 3);
  v7 = create_wall(posv7.x , 4, posv7.x , 6);
  v8 = create_wall(posv8.x , 1, posv8.x , 3);
  v9 = create_wall(posv9.x , 1, posv9.x , 2);
  v10 = create_wall(posv10.x , 1, posv10.x , 4);
  v11 = create_wall(posv11.x , 6, posv11.x , 7);
  v12 = create_wall(posv12.x , 7, posv12.x , 8);
  
  h1 = create_wall(0, posh1.y , 3, posh1.y );
  h2 = create_wall(4, posh2.y , 5, posh2.y );
  h3 = create_wall(6, posh3.y , 7, posh3.y );
  h4 = create_wall(1, posh4.y , 2, posh4.y );
  h5 = create_wall(2, posh5.y , 3, posh5.y );
  h6 = create_wall(4, posh6.y , 6, posh6.y );
  h7 = create_wall(1, posh7.y , 3, posh7.y );
  h8 = create_wall(4, posh8.y , 7, posh8.y );
  h9 = create_wall(0, posh9.y , 2, posh9.y );
  h10 = create_wall(4, posh10.y , 7, posh10.y );
  h11 = create_wall(4, posh11.y , 7, posh11.y );
  h12 = create_wall(2, posh12.y , 3, posh12.y );
  h13 = create_wall(3, posh13.y , 4, posh13.y );
  
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
  v12.setStroke(color(0));
  
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
  
  /* create zones and switch mechanics */
  triggertop = createShape(RECT, deviceOrigin.x + 5*pixelsPerMeterh+20, deviceOrigin.y + 2*pixelsPerMeterv+20, pixelsPerMeterh-40, pixelsPerMeterv-40);
  triggertop.setFill(color(200,0,50));
  triggerbot = createShape(RECT, deviceOrigin.x + 2*pixelsPerMeterh+20, deviceOrigin.y + 7*pixelsPerMeterv+20, pixelsPerMeterh-40, pixelsPerMeterv-40);
  triggerbot.setFill(color(0,50,200));
  //create fake walls
  faketop = create_wall(posftop.x , 3, posftop.x , 4);
  fakebot = create_wall(3, posfbot.y , 4,posfbot.y);
  fakebot.setStroke(color(0,50,200));
  faketop.setStroke(color(200,0,50));
  faketop.setStrokeWeight(24);
  fakebot.setStrokeWeight(24);

// create finish
  finish = createShape(RECT, deviceOrigin.x + 6*pixelsPerMeterh+20, deviceOrigin.y + 6*pixelsPerMeterv+20, pixelsPerMeterh-40, pixelsPerMeterv-40);
  finish.setFill(color(100,50,100));

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

      

      //converstion to 140x80 coordinates, reasonfor +3.5 is the origin of EE is thought to be 0,0
      //but to our cordinate system, it is in fact 3.5,0
      posEEsc.set(posEE.x*pixelsPerMeterEE/pixelsPerMeterh+3.5,posEE.y*pixelsPerMeterEE/pixelsPerMeterv);
      float temp, temp2;
      //see if the ee has reached any trigger
      if (posEEsc.x >5 && posEEsc.x <6 && posEEsc.y < 3 && posEEsc.y >2){
        flagtop = true;
      }
      if (posEEsc.x >2 && posEEsc.x <3 && posEEsc.y < 8 && posEEsc.y >7 ){
        flagbot = true;
      }

      // change along with the flag condition:
      if(flagtop){
      triggertop.setFill(color(0,0,0));
      }
      if(flagbot){
      triggerbot.setFill(color(0,0,0));
      }

//configuring walls for y
      if (posEEsc.y < 2.0){
        penWall.set(abs(posv1.x- posEEsc.x)<rEEx ? (posv1.x>posEEsc.x ? (posv1.x- posEEsc.x-rEEx):(-posv1.x+posEEsc.x+rEEx)):(
          abs(posv4.x- posEEsc.x)<rEEx ? (posv4.x>posEEsc.x ? (posv4.x- posEEsc.x-rEEx):(-posv4.x+posEEsc.x+rEEx)):(
            abs(posv6.x- posEEsc.x)<rEEx ? (posv6.x>posEEsc.x ? (posv6.x- posEEsc.x-rEEx):(-posv6.x+posEEsc.x+rEEx)):(
              abs(posv8.x- posEEsc.x)<rEEx ? (posv8.x>posEEsc.x ? (posv8.x- posEEsc.x-rEEx):(-posv8.x+posEEsc.x+rEEx)):(
                abs(posv9.x- posEEsc.x)<rEEx ? (posv9.x>posEEsc.x ? (posv9.x- posEEsc.x-rEEx):(-posv9.x+posEEsc.x+rEEx)): 0
              )
            )
          )
        ),0);
      }else if(posEEsc.y < 3.0){
        penWall.set(abs(posv1.x- posEEsc.x)<rEEx ? (posv1.x>posEEsc.x ? (posv1.x- posEEsc.x-rEEx):(-posv1.x+posEEsc.x+rEEx)):(
          abs(posv2.x- posEEsc.x)<rEEx ? (posv2.x>posEEsc.x ? (posv2.x- posEEsc.x-rEEx):(-posv2.x+posEEsc.x+rEEx)):(
            abs(posv4.x- posEEsc.x)<rEEx ? (posv4.x>posEEsc.x ? (posv4.x- posEEsc.x-rEEx):(-posv4.x+posEEsc.x+rEEx)):(
              abs(posv6.x- posEEsc.x)<rEEx ? (posv6.x>posEEsc.x ? (posv6.x- posEEsc.x-rEEx):(-posv6.x+posEEsc.x+rEEx)):(
                abs(posv8.x- posEEsc.x)<rEEx ? (posv8.x>posEEsc.x ? (posv8.x- posEEsc.x-rEEx):(-posv8.x+posEEsc.x+rEEx)):(
                  abs(posv10.x- posEEsc.x)<rEEx ? (posv10.x>posEEsc.x ? (posv10.x- posEEsc.x-rEEx):(-posv10.x+posEEsc.x+rEEx)): 0
                  )
                  
                )
              )
            )
          ),0);
      }else if(posEEsc.y < 4.0){
        penWall.set(abs(posv1.x- posEEsc.x)<rEEx ? (posv1.x>posEEsc.x ? (posv1.x- posEEsc.x-rEEx):(-posv1.x+posEEsc.x+rEEx)):(
          abs(posv2.x- posEEsc.x)<rEEx ? (posv2.x>posEEsc.x ? (posv2.x- posEEsc.x-rEEx):(-posv2.x+posEEsc.x+rEEx)):(
            abs(posv10.x- posEEsc.x)<rEEx ? (posv10.x>posEEsc.x ? (posv10.x- posEEsc.x-rEEx):(-posv10.x+posEEsc.x+rEEx)): (
                    flagtop ? 0 : abs(posftop.x- posEEsc.x)<rEEx ? (posftop.x>posEEsc.x ? (posftop.x- posEEsc.x-rEEx):(-posftop.x+posEEsc.x+rEEx)) : 0)
            )
          ),0);
      }else if(posEEsc.y < 5.0){
        penWall.set(abs(posv1.x- posEEsc.x)<rEEx ? (posv1.x>posEEsc.x ? (posv1.x- posEEsc.x-rEEx):(-posv1.x+posEEsc.x+rEEx)):(
          abs(posv5.x- posEEsc.x)<rEEx ? (posv5.x>posEEsc.x ? (posv5.x- posEEsc.x-rEEx):(-posv5.x+posEEsc.x+rEEx)):(
            abs(posv7.x- posEEsc.x)<rEEx ? (posv7.x>posEEsc.x ? (posv7.x- posEEsc.x-rEEx):(-posv7.x+posEEsc.x+rEEx)): 0
            )
          ),0);
      }else if(posEEsc.y < 6.0){
        penWall.set(abs(posv3.x- posEEsc.x)<rEEx ? (posv3.x>posEEsc.x ? (posv3.x- posEEsc.x-rEEx):(-posv3.x+posEEsc.x+rEEx)):(
          abs(posv5.x- posEEsc.x)<rEEx ? (posv5.x>posEEsc.x ? (posv5.x- posEEsc.x-rEEx):(-posv5.x+posEEsc.x+rEEx)):(
            abs(posv7.x- posEEsc.x)<rEEx ? (posv7.x>posEEsc.x ? (posv7.x- posEEsc.x-rEEx):(-posv7.x+posEEsc.x+rEEx)): 0
            )
          ),0);
      }else if(posEEsc.y < 7.0){
        penWall.set(abs(posv3.x- posEEsc.x)<rEEx ? (posv3.x>posEEsc.x ? (posv3.x- posEEsc.x-rEEx):(-posv3.x+posEEsc.x+rEEx)):(
          abs(posv5.x- posEEsc.x)<rEEx ? (posv5.x>posEEsc.x ? (posv5.x- posEEsc.x-rEEx):(-posv5.x+posEEsc.x+rEEx)):(
            abs(posv11.x- posEEsc.x)<rEEx ? (posv11.x>posEEsc.x ? (posv11.x- posEEsc.x-rEEx):(-posv11.x+posEEsc.x+rEEx)): 0
          )
          ),0);
      }else if(posEEsc.y < 8.0){
        penWall.set(abs(posv3.x- posEEsc.x)<rEEx ? (posv3.x>posEEsc.x ? (posv3.x- posEEsc.x-rEEx):(-posv3.x+posEEsc.x+rEEx)):(
          abs(posv5.x- posEEsc.x)<rEEx ? (posv5.x>posEEsc.x ? (posv5.x- posEEsc.x-rEEx):(-posv5.x+posEEsc.x+rEEx)):(
            abs(posv12.x- posEEsc.x)<rEEx ? (posv12.x>posEEsc.x ? (posv12.x- posEEsc.x-rEEx):(-posv12.x+posEEsc.x+rEEx)): 0
          )
          ),0);
      };
      temp = penWall.x;

//horizontal walls configurations now
      if (posEEsc.x < 1.0){
        penWall.set(0,abs(posh1.y- posEEsc.y)<rEEy ? (posh1.y>posEEsc.y ? (posh1.y- posEEsc.y-rEEy):(-posh1.y+posEEsc.y+rEEy)):(
          abs(posh9.y- posEEsc.y)<rEEy ? (posh9.y>posEEsc.y ? (posh9.y- posEEsc.y-rEEy):(-posh9.y+posEEsc.y+rEEy)): 0
        ));
      }else if (posEEsc.x < 2.0){
        penWall.set(0,abs(posh1.y- posEEsc.y)<rEEy ? (posh1.y>posEEsc.y ? (posh1.y- posEEsc.y-rEEy):(-posh1.y+posEEsc.y+rEEy)):(
          abs(posh4.y- posEEsc.y)<rEEy ? (posh4.y>posEEsc.y ? (posh4.y- posEEsc.y-rEEy):(-posh4.y+posEEsc.y+rEEy)): (
            abs(posh7.y- posEEsc.y)<rEEy ? (posh7.y>posEEsc.y ? (posh7.y- posEEsc.y-rEEy):(-posh7.y+posEEsc.y+rEEy)): (
              abs(posh9.y- posEEsc.y)<rEEy ? (posh9.y>posEEsc.y ? (posh9.y- posEEsc.y-rEEy):(-posh9.y+posEEsc.y+rEEy)): 0
            )
          )
        ));
      }else if (posEEsc.x < 3.0){
        penWall.set(0,abs(posh1.y- posEEsc.y)<rEEy ? (posh1.y>posEEsc.y ? (posh1.y- posEEsc.y-rEEy):(-posh1.y+posEEsc.y+rEEy)):(
          abs(posh5.y- posEEsc.y)<rEEy ? (posh5.y>posEEsc.y ? (posh5.y- posEEsc.y-rEEy):(-posh5.y+posEEsc.y+rEEy)): (
            abs(posh7.y- posEEsc.y)<rEEy ? (posh7.y>posEEsc.y ? (posh7.y- posEEsc.y-rEEy):(-posh7.y+posEEsc.y+rEEy)): (
              abs(posh12.y- posEEsc.y)<rEEy ? (posh12.y>posEEsc.y ? (posh12.y- posEEsc.y-rEEy):(-posh12.y+posEEsc.y+rEEy)): 0
            )
          )
        ));
      }else if (posEEsc.x < 4.0){
        penWall.set(0,
              abs(posh13.y- posEEsc.y)<rEEy ? (posh13.y>posEEsc.y ? (posh13.y- posEEsc.y-rEEy):(-posh13.y+posEEsc.y+rEEy)): (
                    flagbot ? 0 : abs(posfbot.y- posEEsc.y)<rEEy ? (posfbot.y>posEEsc.y ? (posfbot.y- posEEsc.y-rEEy):(-posfbot.y+posEEsc.y+rEEy)) : 0)  
          );
      }else if (posEEsc.x < 5.0){
        penWall.set(0,abs(posh2.y- posEEsc.y)<rEEy ? (posh2.y>posEEsc.y ? (posh2.y- posEEsc.y-rEEy):(-posh2.y+posEEsc.y+rEEy)):(
          abs(posh6.y- posEEsc.y)<rEEy ? (posh6.y>posEEsc.y ? (posh6.y- posEEsc.y-rEEy):(-posh6.y+posEEsc.y+rEEy)): (
            abs(posh8.y- posEEsc.y)<rEEy ? (posh8.y>posEEsc.y ? (posh8.y- posEEsc.y-rEEy):(-posh8.y+posEEsc.y+rEEy)): (
              abs(posh10.y- posEEsc.y)<rEEy ? (posh10.y>posEEsc.y ? (posh10.y- posEEsc.y-rEEy):(-posh10.y+posEEsc.y+rEEy)): (
                abs(posh11.y- posEEsc.y)<rEEy ? (posh11.y>posEEsc.y ? (posh11.y- posEEsc.y-rEEy):(-posh11.y+posEEsc.y+rEEy)): 0
              )
            )
          )
        ));
      }else if (posEEsc.x < 6.0){
        penWall.set(0,abs(posh6.y- posEEsc.y)<rEEy ? (posh6.y>posEEsc.y ? (posh6.y- posEEsc.y-rEEy):(-posh6.y+posEEsc.y+rEEy)):(
          abs(posh8.y- posEEsc.y)<rEEy ? (posh8.y>posEEsc.y ? (posh8.y- posEEsc.y-rEEy):(-posh8.y+posEEsc.y+rEEy)): (
              abs(posh10.y- posEEsc.y)<rEEy ? (posh10.y>posEEsc.y ? (posh10.y- posEEsc.y-rEEy):(-posh10.y+posEEsc.y+rEEy)): (
                abs(posh11.y- posEEsc.y)<rEEy ? (posh11.y>posEEsc.y ? (posh11.y- posEEsc.y-rEEy):(-posh11.y+posEEsc.y+rEEy)): 0
                )
              
          )
        ));
      }else if (posEEsc.x < 7.0){
        penWall.set(0,abs(posh3.y- posEEsc.y)<rEEy ? (posh3.y>posEEsc.y ? (posh3.y- posEEsc.y-rEEy):(-posh3.y+posEEsc.y+rEEy)):(
          abs(posh8.y- posEEsc.y)<rEEy ? (posh8.y>posEEsc.y ? (posh8.y- posEEsc.y-rEEy):(-posh8.y+posEEsc.y+rEEy)): (
              abs(posh10.y- posEEsc.y)<rEEy ? (posh10.y>posEEsc.y ? (posh10.y- posEEsc.y-rEEy):(-posh10.y+posEEsc.y+rEEy)): (
                abs(posh11.y- posEEsc.y)<rEEy ? (posh11.y>posEEsc.y ? (posh11.y- posEEsc.y-rEEy):(-posh11.y+posEEsc.y+rEEy)): 0
              )
          )
        ));
      };
      temp2 = penWall.y;
      penWall.set(temp,temp2);
      println(penWall);

         
      //finding force
        fWall = fWall.add(penWall.mult(-kWall));  
      
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
  endEffector = createShape(ELLIPSE, deviceOrigin.x+worldPixelWidth/2, deviceOrigin.y, 2*rEEx*pixelsPerMeterh, 2*rEEy*pixelsPerMeterv);
  endEffector.setStroke(color(0));
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
  
  float lAni = pixelsPerMeterh * l;
  float LAni = pixelsPerMeterv * L;
  
  xE = pixelsPerMeterEE * xE;
  yE = pixelsPerMeterEE * yE;
  
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
  shape(v12);
  if(!flagtop){
    shape(triggertop);
    shape(faketop);
  }
  if(!flagbot){
    shape(triggerbot);
    shape(fakebot);
  }
  
  
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




 