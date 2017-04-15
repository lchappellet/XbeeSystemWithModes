void GpslatLonofDesiredLocation(float Gpslat, float Gpslon, float bearing, float Distance1 )
{
 r_CurLon = radians(Gpslon);
  r_CurLat = radians(Gpslat);
  r_Bearing = radians(bearing);
  float DestLat = asin(sin(r_CurLat)*cos(Distance1/Eradius)+cos(r_CurLat)*sin(Distance1/Eradius)*cos(r_Bearing)); 
  float DestLon = r_CurLon + atan2(sin(r_Bearing)*sin(Distance1/Eradius)*cos(r_CurLat),cos(Distance1/Eradius)-sin(r_CurLat)*sin(DestLat)); 
  DestLon = (DestLon+3*PI)/(2*PI);
  int i = DestLon;
  DestLon = (DestLon - i) * (2*PI) - PI;  // normalise to -180..+180º 

  GpsLat3 = degrees(DestLat);
  GpsLon3 = degrees(DestLon);

  
 /* 
//System.out.printf("starting at a Longitude of %f and a Latitude of %f ",CurLon,CurLat); 
  Serial.print("starting at a Longitude of ");
  Serial.print(CurLon,6); //this gives up to 6 decimal places for the variable CurLon
  Serial.print(" and a Latitude of ");
  Serial.println(CurLat,6);
//System.out.printf("if we travel %f km on a bearing of %f degrees ",Distance,Bearing); 
  Serial.print("if we travel ");
  Serial.print(Distance,6);
  Serial.print(" km on a bearing of ");
  Serial.print(Bearing,6);
  Serial.println(" degrees");
//System.out.printf("we end up at Longitude of %f and a Latitude of %f ",degrees(DestLon),degrees(DestLat)); 
  Serial.print("we end up at a Longitude of ");
  Serial.print(degrees(DestLon),6);
  Serial.print(" and a Latitude of ");
  Serial.println(degrees(DestLat),6);
  */
  //return //***look at using pointers here to get two outputs for two different variables
} 

