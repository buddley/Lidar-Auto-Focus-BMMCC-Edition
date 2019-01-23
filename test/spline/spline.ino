#include <spline.h>

Spline tempCurve;

void setup(void) {
  Serial.begin(115200);
  
  unsigned int scale[14] = {44,45,50,60,70,80,100,120,150,200,300,500,5357,5358};
  unsigned int pos[14] = {999,1000,1075,1275,1395,1445,1505,1555,1605,1655,1705,1755,1830,1831};

  float x[14], y[14];
    for (int i = 0; i < 14; i++){
        x[i] = scale[i];
        y[i] = pos[i];
    }

  tempCurve.setPoints(x,y,14);
  tempCurve.setDegree( Catmull );
  
  for( int i = 45; i <= 5357; i+= 20 ) {
    float temp = tempCurve.value(float(i));
    Serial.print(i);
    for(float j=0; j<= temp; j += 20) {
      Serial.print( "*" );
    }
    Serial.print( "   " );
    Serial.println(int(temp));
  }
}

void loop(void) {

}
