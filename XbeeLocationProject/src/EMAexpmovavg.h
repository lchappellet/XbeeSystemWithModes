/*
 * EMAexpmovavg.h
 *
 *  Created on: Apr 22, 2017
 *      Author: London
 */

#ifndef EMAEXPMOVAVG_H_
#define EMAEXPMOVAVG_H_


class EMA_exp_mov_avg {
public:
		//N = length of EMA
		// ema_value_now = Value of data you have just taken now.
		// ema_previous = EMA previously taken
		// ema_current  = EMA calculated due to newest Value
	    // Formula for EMA: ema_current = (ema_value_now * K) + (ema_previous*(1-K))
	int N =0;
	double ema_value_now =0;
	double ema_previous = 0;
	double ema_current =0;
	double K = 2/(N+1);

public:
	EMA_exp_mov_avg();
	virtual ~EMA_exp_mov_avg();
	void EMA_start_set_values(int length_of_sample, double first_ema_current_value);
	double EMA_run();
	void EMA_store_values();
};

#endif /* EMAEXPMOVAVG_H_ */
