/* ==============================================================================
 *  Program created by Riccardo Bravin and Francesco Caserta
==============================================================================*/

//#define DEBUG 1

#include <TensorFlowLite.h>
#include <Arduino_LSM9DS1.h> //for IMU
#include <arduinoFFT.h>
#include <Arduino_KNN.h>
//#include <ArduinoLowPower.h>


#include "model.h"  //contains model data
#include "result_visualizer.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"


#define MAX_SAMPLES 512
#define SAMPLING_FREQUENCY 238

#define OUTPUT_CLASSES 128

#define LED_PIN 13
#define PW_LED_PIN 25

//model variables
namespace {
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

constexpr int kTensorArenaSize = 82*1024; 

alignas(16) uint8_t tensor_arena[kTensorArenaSize]; // Keep aligned to 16 bytes for CMSIS
}  // namespace


//arrays for data and FFT
double data[6][MAX_SAMPLES];
double imagAux[6][MAX_SAMPLES];
arduinoFFT FFTs[6]; //array of FFT objects


//training mode handling variables
bool trainMode = false;
int exampleToAdd = 5; //number of samples to record ad each recording session
int counter; //counter of remaining samples to record
int newLabel; //label inserted by user


//sleep handling variables
double power = 0;
float thresholdPower = 0.3; //threshold for power
unsigned char silenceCount = 0;

  
//---------------------------------------------------------------- KNN setup  ----------------------------------------------------------------//
// create KNN classifier, input will be array of 128 floats
KNNClassifier myKNN(128);


void setup() {
  
  //---------------------------------------------------------------- IMU setup  ----------------------------------------------------------------//
  if (!IMU.begin()) {
    MicroPrintf("Failed to initialize IMU!");
    while (1);
  }

  /******  FS  Full Scale         range 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)                                           ******
  ******                          range 0: 245 dps | 1: 500 dps | 2: 1000  dps | 3: 2000 dps                                    ******
  *******  ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz) ******/
  IMU.setAccelFS(0);   //best accuracy at 0
  IMU.setAccelODR(4);  //best refreshrate at 6 (if Gyro OFF?)
  IMU.setGyroFS(0);
  IMU.setGyroODR(4);

  /*******************    For an improved accuracy run the DIY_Calibration_Accelerometer sketch first.     *****************/
  IMU.setAccelOffset(0, 0, 0);  //uncalibrated
  IMU.setAccelSlope(1, 1, 1);   //uncalibrated
  IMU.setGyroOffset(0, 0, 0);   //uncalibrated
  IMU.setGyroSlope(1, 1, 1);    //uncalibrated

  IMU.accelUnit = GRAVITY;         // GRAVITY=1 or  METERPERSECOND2=9.81
  IMU.gyroUnit = DEGREEPERSECOND;  // DEGREEPERSECOND  RADIANSPERSECOND  REVSPERMINUTE  REVSPERSECOND

  
  //---------------------------------------------------------------- CNN setup  ----------------------------------------------------------------//
  tflite::InitializeTarget();

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model provided is schema version %d not equal to supported version %d.", model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // This pulls in all the operation implementations we need.
  //static tflite::AllOpsResolver resolver;
  static tflite::MicroMutableOpResolver<10> resolver; 
  resolver.AddConv2D();
  resolver.AddRelu();
  resolver.AddFullyConnected();
  resolver.AddSoftmax();
  resolver.AddReshape();
  resolver.AddQuantize();
  resolver.AddMul();
  resolver.AddAdd();
  resolver.AddMean();
  resolver.AddDequantize();
  
  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  // Obtain pointers to the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);

  Serial.begin(9600); //inserito per prendere dati da input
  Serial.setTimeout(5);
  while (!Serial);

  //---------------------------------------------------------------- FFT setup  ----------------------------------------------------------------//

  //inizializzo FFTs objects by passing all reference 
  for(int i = 0; i < 6; i++){
    FFTs[i] = arduinoFFT(data[i], imagAux[i], MAX_SAMPLES, SAMPLING_FREQUENCY);
  }
  
  //---------------------------------------------------------------- Misc setup  ----------------------------------------------------------------//

  // initialize the LED as an output:
  pinMode(LED_PIN, OUTPUT);
  pinMode(PW_LED_PIN, OUTPUT);
  
}


void loop() {

  //define variables for IMU reads
  int16_t acc[3], gyro[3];
  
  
  //if class 1 (silence) is detected more than 5 times go to sleep
  if(silenceCount > 8){
    MicroPrintf("-----------SLEEP MODE-----------");
    digitalWrite(PW_LED_PIN, LOW);
    digitalWrite(22,HIGH);
    digitalWrite(23,HIGH);
    digitalWrite(24,HIGH);
    
    power = 0;
    unsigned long sampCount = 0;
    
    //turn of gyro and set accel to 10Hz to save power
    IMU.setAccelODR(1);
    IMU.setGyroODR(0);

    //stay in sleep until power is higher than threshold
    do{

      if (IMU.accelAvailable()) {
        //read only accelerometer data
        IMU.readRawAccelArr(acc);

        sampCount ++;
        double aux = ((double)(acc[0]) / 16384)* ((double)(acc[0]) / 16384);
        //power += (aux - power)/sampCount;
        power = aux;
        
        sampCount ++;
        aux = ((double)(acc[1]) / 16384)* ((double)(acc[1]) / 16384);
        //power += (aux - power)/sampCount;
        power += aux;
        
        sampCount ++;
        aux = ((double)(acc[2]) / 16384)* ((double)(acc[2]) / 16384);
        //power += (aux - power)/sampCount;
        power += aux;

        power /=3;

        //LowPower.idle(95);//sleep for 95ms to get 10Hz refresh rate
        delay(200);

        #ifdef DEBUG
          Serial.print("Power: ");
          Serial.println(power,20);
          Serial.print("Threshold: ");
          Serial.println(thresholdPower,20);
        #endif
      }

    }while(power < thresholdPower || sampCount < 61);

    silenceCount = 0;
    digitalWrite(PW_LED_PIN, HIGH);
    IMU.setAccelODR(4);
    IMU.setGyroODR(4);
  }

  char inputStr[16] = ""; 
  Serial.readBytesUntil('\n', inputStr, 15);

  //if command passed, enter train mode
  if(inputStr[0] != 0){
    trainMode = true; //set train mode flag
    newLabel = inputStr[0] - '1' + 1;

    counter = exampleToAdd;//reset counter

    MicroPrintf("--------------------\n STARTED TRAIN MODE\n--------------------");
  }
  
  MicroPrintf("-----------Acquiring data-----------");
  unsigned long startTimeAcquisition = millis();

  // Read samples until it gets to the MAX_SAMPLES value
  int sample_count = 0;
  power = 0;
  while (sample_count < MAX_SAMPLES) {

    if (IMU.accelAvailable() && IMU.gyroAvailable()) {
      //read both accelerometer and gyro data
      IMU.readRawAccelArr(acc);
      IMU.readRawGyroArr(gyro);

      //Converti i valori letti nell'intervallo desiderato per il modello
      data[0][sample_count] = (double)(acc[0]) / 16384; //accx
      data[1][sample_count] = (double)(acc[1]) / 16384; //accy
      data[2][sample_count] = (double)(acc[2]) / 16384; //accz
      data[3][sample_count] = (double)(gyro[0]) / 16384; //gyrox
      data[4][sample_count] = (double)(gyro[1]) / 16384; //gyroy
      data[5][sample_count] = (double)(gyro[2]) / 16384; //gyroz

      //resetting imaginary component for FFT
      imagAux[0][sample_count] = 0; imagAux[1][sample_count] = 0; imagAux[2][sample_count] = 0; imagAux[3][sample_count] = 0; imagAux[4][sample_count] = 0; imagAux[5][sample_count] = 0;
      
      if(sample_count % 64 < 32)
        digitalWrite(LED_PIN, HIGH);
      else{
        digitalWrite(LED_PIN, LOW);
      }
      
      #ifdef DEBUG
        Serial.print((double)(acc[2]) / 16384); //prints z axis 
        Serial.print(", ");
      #endif

      power += data[0][sample_count]*data[0][sample_count] + 
               data[1][sample_count]*data[1][sample_count] + 
               data[2][sample_count]*data[2][sample_count];

      sample_count++;
    }
  }
  //divide power by the number of samples for later
  power /= (MAX_SAMPLES * 3);
  
  #ifdef DEBUG
    Serial.println("");

    for (int j = 0; j < (MAX_SAMPLES >> 1); j++) {
        Serial.print(data[2][j]);
        Serial.print(", ");
    }

    Serial.println("");
  
    Serial.print("Current power measured: ");
    Serial.println(power, 20);
  #endif
  
  MicroPrintf("Duration: %u\n", millis()-startTimeAcquisition);
  
  MicroPrintf("-----------Performing FFT-----------");
  unsigned long startTimeFFT = millis();

  
  //Intrinsically the FFT object performs the transform on the data matrix
  for(int i = 0; i < 6; i++){
    FFTs[i].Compute(FFT_FORWARD);
    FFTs[i].ComplexToMagnitude();
  }

  #ifdef DEBUG
    for (int j = 0; j < (MAX_SAMPLES >> 1); j++) {
      Serial.print(static_cast<float>(data[0][j]));
      Serial.print(", ");
    }

    Serial.println("");
  #endif
  
  MicroPrintf("Duration: %u", millis()-startTimeFFT);


  MicroPrintf("-----------CNN inference-----------");
  unsigned long startTimeInference = millis();

  int input_index = 0;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < (MAX_SAMPLES >> 1); j++) {
      input->data.f[input_index] = static_cast<float>(data[i][j]);
      input_index++;
    }
  }


  
  // Run inference, and report any error
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    MicroPrintf("Invoke failed on data array");
    return;
  }

  MicroPrintf("Duration: %u\n", millis()-startTimeInference);


  MicroPrintf("-----------KNN inference/training-----------");
  unsigned long startTimeKNN = millis();
  
  // Obtain the output from model's output tensor
  float *y = output->data.f;

  #ifdef DEBUG
    for(int i = 0; i < 128; i++){
      Serial.print(y[i]);
      Serial.print(", ");
    }
    Serial.println("");
  #endif
  
  

  if(trainMode){ //TRAIN MODE
    myKNN.addExample(y, newLabel); //add example to KNN classifier

    MicroPrintf("Added %d/%d samples for class %d", exampleToAdd - counter + 1, exampleToAdd, newLabel);
    counter--;
    
    if(counter==0){ //if all examples have been added, exit train mode
      trainMode = false;
      MicroPrintf("Returning to INFERENCE MODE");
    }
    

  }else{ //CLASSIFY MODE
    int personLabel = myKNN.classify(y, 10); //k=5. provo con k=sqrt(120)

    if(personLabel > 0){ //note: labels must be >0
      Serial.print("Predicted class ");
      Serial.print(personLabel);
      Serial.print(" with ");
      Serial.print(myKNN.confidence());
      Serial.println(" confidence");
      //MicroPrintf("Predicted class %d with %f confidence", personLabel, myKNN.confidence());
      class_to_led(personLabel, true);
    }
    else{
      MicroPrintf("Not enough samples for inference");
    }

    if(personLabel == 1){
      thresholdPower =  thresholdPower * 0.9 + power * 0.1;
      silenceCount++;
    }else{
      silenceCount = 0;
    }

  }

  MicroPrintf("Duration: %u\n", millis()-startTimeKNN);
  
}
