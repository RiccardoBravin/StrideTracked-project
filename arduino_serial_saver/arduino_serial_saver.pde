
import processing.serial.*;
import java.io.FileWriter;

Serial arduPort;  // The serial port
File fileOut; //file to which to output
FileWriter output; //printer on fileOut

Graph[] acc = new Graph[3];
Graph[] gir = new Graph[3];

int label = -1;


void setup() {

  size(800, 400);
  surface.setTitle("Data visualizer");
  surface.setResizable(true);

  strokeWeight(2); //thickness of lines
  noFill();

  // List all the available serial ports
  printArray(Serial.list());

  // Open the port you are using at the baudrate of the device
  if (Serial.list().length == 0) {
    print("No used ports available\nExiting...\n");
    System.exit(0);
  }

  //open the serial port
  arduPort = new Serial(this, Serial.list()[0], 57600);

  //fill graph arrays
  acc[0] = new Graph(color(250, 90, 90), 150);
  acc[1] = new Graph(color(90, 250, 90), 150);
  acc[2] = new Graph(color(90, 90, 250), 150);
  gir[0] = new Graph(color(250, 90, 90), 50);
  gir[1] = new Graph(color(90, 250, 90), 50);
  gir[2] = new Graph(color(90, 90, 250), 50);

  // Throw out the first few reading, in case we started reading in the middle of a string from the sender.
  for (int i = 0; i < 10; i++) {
    arduPort.clear();
  }
  arduPort.readStringUntil('\n');
}

void draw() {
  background(0);

  String dataRead = "";
  //SERIAL PORT READER AND PARSER
  while (arduPort.available() > 0) {
    //read string up to the endline
    dataRead = arduPort.readStringUntil('\n');
    
    if (dataRead != null) {
      
      String line = dataRead.substring(0,dataRead.length()-1).concat("\t");
      
      line = line.concat(str(label)).concat("\n");
      
      print(line);

      try {
        output.write(line);
      }
      catch(Exception e) {
        println("cannot write to file");
      }
    }
  }
  
  //SAVE AND DRAW GRAPH ON SOME DATA
  //split string read into
  float[] nums = float(split(dataRead, '\t'));

  //Fill graph object with data
  acc[0].insert(nums[0]);
  acc[1].insert(nums[1]);
  acc[2].insert(nums[2]);
  gir[0].insert(nums[3]);
  gir[1].insert(nums[4]);
  gir[2].insert(nums[5]);

  //graph drawing

  acc[0].display(0);
  acc[1].display(0);
  acc[2].display(0);
  gir[0].display(1);
  gir[1].display(1);
  gir[2].display(1);
}

//Function to end reads also used to switch the label
void keyPressed() {
  if (key == ' ') {
    // Clear the buffer of
    arduPort.clear();
    // Close the port
    arduPort.stop();
    try {
      output.flush();
      output.close();
    }
    catch (Exception e) {
      println(e);
    }
    exit();
  }

  if ((key >= '0' && key <= '9') || (key >= 'a' && key <= 'z')) {
    
    label = int(key-'a'+10);
    if(label < 0) 
      label = int(key-'0');
      
    try {
      output.flush();
      output.close();
    }
    catch(Exception e) {
    }
    try {
      int x = 0;
      do {
        //change path to other in case of errors
        fileOut = new File("C:\\Users\\Riccardo Bravin\\Desktop\\ardu project\\arduino_serial_saver\\samples\\label" + label + "_" + x + ".csv");
        //fileOut = new File("C:\\Users\\caser\\Documents\\POLIMI\\HW-EDGE-AI\\projectWalk\\DataCollected\\label" + label + "_" + x + ".csv");
        x++;
      } while (!fileOut.createNewFile());

      output = new FileWriter(fileOut);
    }
    catch(Exception e) {
    }
  }
}
