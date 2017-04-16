/*
 * HeadingCalibCalculation.h
 *
 *  Created on: Apr 8, 2017
 *      Author: London
 */

#ifndef HEADINGCALIBCALCULATION_H_
#define HEADINGCALIBCALCULATION_H_

class HeadingCalibCalculation {
	void CalibHeadToPosition(){
	   readEncoders();
	   currentPos = cartCurrent[0];
	 NowAngle = headingDegrees;

	float Num = NowAngle-currentPos;
	if(Num >=0 ){                               ////////// *****************not finished here!!!!! still need to complete
	     int Total = Num;
	     int TotalPlus180 = 180+total;

	     if(TotalPlus180 > 360){
	           int NewTotal = 360 - TotalPlus180;

	        }

	     else{
	          NewTotal = TotalPlus180
	         }
	         if(AverageAngle>=180){
	           int Degrees = map(AverageAngle, Total, 360, 0, abs(Num));
	         }

	           else {
	         int Degrees = map(AveragedAngle,0,NewTotal,0,NewTotal);
	     }



	     }

	  }
	  if(Num < 0){
	    Total = 360+Num;
	    TotalPlus180 = Total+180;
	      if(TotalPlus180 > 360){
	        int NewTotal = 360 - TotalPlus180;
	         if(headingDegrees>Total){
	                int Degrees = map(headingDegrees, Total, 360, 0, abs(Num));  ////////// *****************not finished here!!!!! still need to complete
	             }
	         else  {
	                int Degrees = map(headingDegrees,0,NewTotal,0,180);
	           }
	     }
	      Total->TotalPlus180
	     TotalPlus180 -> NewTotal
	     TotalPlus180


	  }
	}

public:
	HeadingCalibCalculation();
	virtual ~HeadingCalibCalculation();
};

#endif /* HEADINGCALIBCALCULATION_H_ */
