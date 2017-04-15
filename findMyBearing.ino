float findMyBearing(float bflat1, float bflon1, float bflat2, float bflon2){

//   var y = Math.sin(Δλ) * Math.cos(φ2);
//    var x = Math.cos(φ1)*Math.sin(φ2) -
//            Math.sin(φ1)*Math.cos(φ2)*Math.cos(Δλ);
//    var θ = Math.atan2(y, x);

//var y = Math.sin(λ2-λ1) * Math.cos(φ2);
//    var x = Math.cos(φ1)*Math.sin(φ2) -
//            Math.sin(φ1)*Math.cos(φ2)*Math.cos(λ2-λ1);
//    var brng = Math.atan2(y, x).toDegrees();

    
  float r_long1 = radians(bflon1);
  float r_long2 = radians(bflon2);
  float r_lat1 = radians(bflat1);
  float r_lat2 = radians(bflat2);
    yCoord = sin(r_long2 - r_long1)*cos(r_lat2);
//  Serial.println(sin(long2 - long1),6);
    Serial.println(yCoord,6);
    xCoord = cos(r_lat1)*sin(r_lat2)- sin(r_lat1)*cos(r_lat2)* cos(r_long2-r_long1);
    Serial.println(xCoord,6);
//    Serial.println(cos(long2 - long1),6);
//    Serial.println(cos(lat2));
   float boatBearing1 = atan2(yCoord,xCoord);
   float degreesboatBearing = boatBearing1*180/3.1459;
    Serial.println("bearing in Degrees :");
    Serial.println(boatBearing1*180/3.1459);
    return degreesboatBearing;
}
