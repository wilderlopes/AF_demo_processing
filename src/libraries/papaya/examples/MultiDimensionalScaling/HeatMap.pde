/** (very rudimentary!) class for drawing a heatmap representring the distance matrix. 
No permuting of rows/columns to reveal structure or anything like that. 
Just takes in a matrix and plots the values as colors.*/

public class HeatMap extends Visuals{
  
  public float wEach = 0; public float hEach = 0;
  /** Constructor */
  public HeatMap(PApplet parent, float xLeft, float yTop, float xWidth, float yHeight){    
    super(parent,xLeft,yTop,xWidth,yHeight);
  }
  
  // input the data and specify the min and max values for the color mapping
  void drawHeatMap(float[][] input, float minVal, float maxVal){
    int numRows = input.length; 
    int numColumns = input[0].length;
    wEach = xWidth/numColumns; hEach = yHeight/numRows;
    
    // transform the distance matrix into a matrix of colors with darker colors
    // corresponding to larger distances. To do this, we call the 'mapToColorScale' function 
    int[][] colors = mapToColorScale(input,minVal,maxVal);
    
    // label the rows
    textFont(tinyFont);
    textAlign(CENTER,CENTER);
    for(int i=0; i<numRows; i++){
      fill(100);
      text(i, xLeft-10, yTop+(i+.5)*hEach);
    }
    for(int j=0; j<numColumns; j++) {
      text(j+1, xLeft + (j+.5)*wEach, yTop+yHeight+5);
    }
    
   for(int i=0; i<numRows; i++){
       for(int j=0; j<numColumns; j++){
          fill(colors[i][j]); noStroke();
          rect(xLeft + j*wEach, yTop+i*hEach,wEach,hEach);
      }
    }
  }
}

// maps the input data from 0 to the specified maximum and returns an array of ints corresponding to the colors
int[][] mapToColorScale(float[][] input, float minVal, float maxVal){
  int numRows = input.length; int numColumns = input[0].length;
  int[][] output = new int[numRows][numColumns];
  for(int i=0; i<numRows; i++){
      for(int j=0; j<numColumns; j++){
        float v = floor(map(input[i][j],minVal,maxVal, 0,200));
        output[i][j] = (int)v; // for more fun, try color(200-v,v,0); or variations thereoff.
      }
      
  }
  return output;
}
