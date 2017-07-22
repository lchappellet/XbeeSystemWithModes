/*
 * EMAexpmovavg.cpp
 *
 *  Created on: Apr 22, 2017
 *      Author: London
 */

#include "EMAexpmovavg.h"

EMA_exp_mov_avg::EMA_exp_mov_avg() {
	// TODO Auto-generated constructor stub

}

EMA_exp_mov_avg::~EMA_exp_mov_avg() {
	// TODO Auto-generated destructor stub
}
void EMA_exp_mov_avg::EMA_start_set_values(double length_of_sample, double first_ema_current_value){
	// This will store the necessary values for the EMA
	// the length_of_sample will give us N which is the total number of values in the average
	// first_ema_current_value is going to give us the first value to start off the Ema

	N = (int)length_of_sample;
	ema_current = first_ema_current_value;




}
void EMA_exp_mov_avg::EMA_store_values(double ema_current_value){
	//This will store the next value in the EMA so that it can then also calculate the next EMA.
	// K = 2/(N+1)
	//N = length of EMA
	// ema_value_now = Value of data you have just taken now.
	// ema_previous = EMA previously taken
	// ema_current  = EMA calculated due to newest Value
    // Formula for EMA: ema_current = (ema_value_now * K) + (ema_previous*(1-K))
	ema_previous = ema_current_value;


}
double EMA_exp_mov_avg::EMA_run(double ema_value_now_now){
	//run the EMA to give return the Exponential Moving average of whatever average is input into the calculator
	ema_current = (K*ema_value_now_now)+ ema_previous*(1-K);

	return ema_current;
}
