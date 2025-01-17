#ifndef REWARD_H
#define REWARD_H

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>


// METHODS
	bool RunReward();
    void ProcRewCmd(byte cmd_type, float cmd_goal = -1, int cmd_zone_delay = -1);
	void SetZoneDur(int zone_ind = -1);
	void SetZoneBounds(float cmd_goal);
	bool CheckZoneBounds();
	void RewardReset(bool was_rewarded = false);


#endif