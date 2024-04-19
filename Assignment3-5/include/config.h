//-----Debug------------------------------------------------------

//See all data communicated over SPI
#define DEBUG_SPI               false
//Printf sleep and timeout time of control loop
#define DEBUG_CONTROL_LOOP      false
//Data of the cross buffer is printed at start
#define DEBUG_X_BUF             false
//Used to debug data received from ROS
#define DEBUG_ROS2XENO          true
//Used to debug data that is to be sent to ROS
#define DEBUG_XENO2ROS          false

//-----Control----------------------------------------------------

//Must be bigger than 1
#define CYCLE_TIME_FREQ         1000        
/*Frequency of which data will be written to ROS, 
Must be lower than CYCLE_TIME_FREQ 
Write frequence will be CYCLE_TIME_FREQ/XENO2ROS_FREQ rounded
If set to 0 no data will be sent to ROS*/
#define XENO2ROS_FREQ           1 
//Enables kill condition for control loop      
#define ENABLE_KILL_CONDITION   true

//-----Communication----------------------------------------------

//Size mutliplier of the amount of ROS2XENO msg in the cross buffer 
#define ROS2XENO_XBUF_SIZE      1000
//Size mutliplier of the amount of the XENO2ROS msg in the cross buffer           
#define XENO2ROS_XBUF_SIZE      1000           

//-----Safety----------------------------------------------------
//Do not change witout considiration of the consequences 

//Enables limiting the output based on MAX_PWM_OUTPUT
#define ENABLE_OUTPUT_LIMIT     true
//The maximum PWM output allowed in %. Has to be between 0 to 1.
#define MAX_PWM_OUTPUT          0.25

//-----20-Sim----------------------------------------------------

//Set to true to take cycle time as defined in 20Sim model
#define OVERWRITE_CYCLE_TIME    true
//Size of the input array of the 20Sim model
#define SIZE_U                  4
//Size of the ouput array of the 20Sim model
#define SIZE_Y                  2
//Debug option to see the input and output of the 20Sim model
#define DEBUG_20SIM_IO          false    