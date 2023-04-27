import java.util.LinkedList;


public class Graph {


  LinkedList<Float> varlist;
  int pixel_per_sample;
  color col;
  int verticalPlacing;
  float range;


  Graph() {
    varlist = new LinkedList();
    pixel_per_sample = 4; 
    col = color(255,255,255);
    range = 0.5;
  }

  Graph(color col, float range) {
    this.varlist = new LinkedList();
    this.pixel_per_sample = 4; 
    this.col = col; 
    this.range = range;
  }

  void insert(float val) {
    varlist.push(val);
    
    if(varlist.size() > width/pixel_per_sample){
      varlist.removeLast();  
    }
  }
  
  void display(int verticalPlacing){
    float avg = 0;
    for (int i = 0; i < varlist.size(); i++) {
      avg += varlist.get(i);
    }
    avg /= varlist.size();
    
    //Draw  
    stroke(this.col); //red for x axis (change at will)
    int pos = 0;
    beginShape();
    for (float y : varlist) {
      vertex(pos, norm(y, avg + range, avg - range)*height/2 + (height*verticalPlacing/2));
      pos += width / varlist.size();
    }
    endShape();
  }
}
