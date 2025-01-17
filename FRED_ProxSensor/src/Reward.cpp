// #include <Arduino.h>
// #include <avr/interrupt.h>
// #include <avr/io.h>
// #include "SafeVector.h"





// static const int zoneLng =
// 		sizeof(_zoneLocs) / sizeof(_zoneLocs[0]);

// // VARIABLES
// 	int durationDefault = 2000; // (ms) 2000 
// 	const int _zoneRewDurs[9] = { // (ms)
// 		100,
// 		182,
// 		284,
// 		368,
// 		400,
// 		368,
// 		284,
// 		182,
// 		100
// 	};
// 	const int _zoneLocs[9] = { // (deg)
// 		20,
// 		15,
// 		10,
// 		5,
// 		0,
// 		-5,
// 		-10,
// 		-15,
// 		-20,
// 	};
// 	const VEC<int> zoneRewDurs;
// 	const VEC<int> zoneLocs;
// 	static const int zoneLng =
// 		sizeof(_zoneLocs) / sizeof(_zoneLocs[0]);
// 	VEC<double> zoneBoundCumMin;
// 	VEC<double> zoneBoundCumMax;
// 	VEC<int> zoneOccTim;
// 	VEC<int> zoneOccCnt;
// 	VEC<double> zoneBoundCumRewarded;
// 	int cnt_cmd = 0;
// 	int cnt_rew = 0;
// 	uint32_t t_nowZoneCheck = 0;
// 	uint32_t t_lastZoneCheck = 0;
// 	int rewDelay = 0; // (ms)
// 	int rewDuration = 0; // (ms) 
// 	float solOpenScale = 1;
// 	int zoneMin = 0;
// 	int zoneMax = 0;
// 	uint32_t t_rewStr = 0;
// 	uint32_t t_rewEnd = 0;
// 	uint32_t t_closeSol = 0;
// 	uint32_t t_retractArm = 0;
// 	uint32_t t_moveArmStr = 0;
// 	double goalPosCum = 0;
// 	bool isRewarding = false;
// 	bool isZoneTriggered = false;
// 	bool isAllZonePassed = false;
// 	bool is_ekfNew = false;
// 	int zoneInd = 0;
// 	int zoneRewarded = 0;
// 	int occRewarded = 0;
// 	int lapN = 0;
// 	uint32_t armMoveTimeout = 5000;
// 	bool do_ArmMove = false;
// 	bool do_ExtendArm = false;
// 	bool do_RetractArm = false;
// 	bool do_TimedRetract = false;
// 	bool isArmExtended = false;
//     bool v_doStepTimer;
//     byte v_stepTarg; 
//     bool v_isArmMoveDone;
//     char v_stepDir;
// 	const int dt_step_high = 500; // (us)
// 	const int dt_step_low = 500; // (us)
// 	bool isArmStpOn = false;
//     const int dt_rewBlock = 30000; // (ms)
//     const int rewZoneWidth = 5; // (deg)

// 	enum REWMODE {
// 		BUTTON,
// 		NOW,
// 		CUE,
// 		FREE,
// 	};
// 	const char *p_str_list_rewMode[4] =
// 	{ { "BUTTON" },{ "NOW" },{ "CUE" },{ "FREE" } };
// 	REWMODE rewMode = BUTTON;
// 	enum MICROSTEP {
// 		FULL,
// 		HALF,
// 		QUARTER,
// 		EIGHTH,
// 		SIXTEENTH
// 	};
// 	const char *p_str_microstep[5] =
// 	{ { "FULL" },{ "HALF" },{ "QUARTER" },{ "EIGHTH" },{ "SIXTEENTH" } };
// 	const MICROSTEP ezExtMicroStep = QUARTER;
// 	const MICROSTEP ezRetMicroStep = QUARTER;

	

// void ProcRewCmd(byte cmd_type, float cmd_goal, int cmd_zone_delay)
// {

// 	// NOTE: arg2 = reward delay or zone ind or reward duration

// 	// Local vars
// 	int cmd_zone_ind = -1;
// 	int cmd_delay = -1;

// 	// Store mode
// 	rewMode =
// 		cmd_type == 0 ? BUTTON :
// 		cmd_type == 1 ? NOW :
// 		cmd_type == 2 ? CUE :
// 		FREE;

// 	// Update counts
// 	if (rewMode != BUTTON)
// 	{
// 		cnt_cmd++;
// 	}
// 	cnt_rew++;

// 	// Format string

// 	// Handle zone/delay arg
// 	if (rewMode == NOW || rewMode == CUE)
// 	{

// 		// Set to zero based index
// 		cmd_zone_ind = cmd_zone_delay - 1;
// 	}
// 	else if (rewMode == FREE)
// 	{
// 		cmd_delay = cmd_zone_delay;
// 	}

// 	// Setup "BUTTON" reward
// 	if (rewMode == BUTTON)
// 	{

// 		// Set duration to default
// 		SetZoneDur();
// 	}

// 	// Setup "NOW" reward
// 	else if (rewMode == NOW)
// 	{

// 		// Set duration
// 		SetZoneDur(cmd_zone_ind);
// 	}

// 	// Setup "CUE" reward
// 	else if (rewMode == CUE)
// 	{

// 		// Include specified zone
// 		zoneMin = cmd_zone_ind;
// 		zoneMax = cmd_zone_ind;

// 		// Set zone bounds
// 		SetZoneBounds(cmd_goal);

// 		// Set delay to zero
// 		rewDelay = 0;
// 	}

// 	// Setup "FREE" reward
// 	else if (rewMode == FREE)
// 	{

// 		// Include all zones
// 		zoneMin = 0;
// 		zoneMax = zoneLng - 1;

// 		// Set zone bounds
// 		SetZoneBounds(cmd_goal);

// 		// Store reward delay time in ms
// 		rewDelay = cmd_delay * 1000;
// 	}

// 	// Log

// }

// bool RunReward()
// {


// 	// Local vars
// 	bool reward_done = false;

// 	// Bail if rewarding
// 	if (isRewarding)
// 	{

// 		return reward_done;
// 	}

// 	// Zone not triggered yet
// 	if (!isZoneTriggered)
// 	{

// 		// Check each zone
// 		if (CheckZoneBounds())
// 		{

// 			// Start reward
// 			StartRew();

// 			// Print message
		

// 			// Set done flag
// 			reward_done = true;

// 		}
// 	}

// 	// Check if rat passed all bounds
// 	if (isAllZonePassed &&
// 		!isZoneTriggered)
// 	{

// 		// Print reward missed

// 		// Send missed reward msg
// 		QueuePacket(&r2c, 'Z', cnt_rew, 0, zoneInd + 1, 0, true);

// 		// Decriment reward count
// 		cnt_rew--;

// 		// Reset flags
// 		RewardReset(reward_done);

// 		// Set done flag
// 		reward_done = true;

// 	}

// 	// Return flag
// 	return reward_done;
// }




// void SetZoneDur(int zone_ind)
// {
	

// 	// Local vars
	

// 	// Set zone ind
// 	if (zone_ind != -1)
// 	{
// 		zoneInd = zone_ind;
// 	}

// 	// Find default ind
// 	else {
// 		for (int i = 0; i < zoneLng; i++)
// 		{
// 			zoneInd = zoneRewDurs[i] == durationDefault ? i : zoneInd;
// 		}
// 	}

// 	// Set duration
// 	rewDuration = zoneRewDurs[zoneInd];

// 	// Log

	
// }

// void SetZoneBounds(float cmd_goal)
// {
	

// 	// Local vars
	
// 	int diam = 0;
// 	int pos_int = 0;
// 	double pos_cum = 0;
// 	double dist_center_cm = 0;
// 	double dist_start_cm = 0;
// 	double dist_end_cm = 0;

// 	// Compute laps
// 	diam = (int)(140 * PI * 100);
// 	pos_int = (int)(RatPos * 100);
// 	lapN = round(RatPos / (140 * PI) - (float)(pos_int % diam) / diam);
// 	// Check if rat 'ahead' of rew pos
// 	pos_cum = (double)(pos_int % diam) / 100;
// 	// Add lap
// 	lapN = pos_cum > cmd_goal ? lapN + 1 : lapN;

// 	// Compute reward center
// 	goalPosCum = cmd_goal + lapN*(140 * PI);

// 	// Compute bounds for each zone
// 	for (int i = zoneMin; i <= zoneMax; i++)
// 	{
// 		// Get zone width with overlap for center bins
// 		int zone_bnd_start = zoneMin == zoneMax || i == zoneMin ?
// 			rewZoneWidth / 2 : rewZoneWidth;
// 		int zone_bnd_end = zoneMin == zoneMax || i == zoneMax ?
// 			rewZoneWidth / 2 : rewZoneWidth;

// 		// Compute zone bounds
// 		dist_center_cm = -1 * zoneLocs[i] * ((140 * PI) / 360);
// 		dist_start_cm = dist_center_cm - (zone_bnd_start * ((140 * PI) / 360));
// 		dist_end_cm = dist_center_cm + (zone_bnd_end * ((140 * PI) / 360));

// 		// Store in array
// 		zoneBoundCumMin[i] = goalPosCum + dist_start_cm;
// 		zoneBoundCumMax[i] = goalPosCum + dist_end_cm;
// 	}

// 	// Print message
	

	
// }

// bool CheckZoneBounds()
// {
	

// 	// Run only if reward not already triggered
// 	if (isZoneTriggered)
// 	{
		
// 		return isZoneTriggered;
// 	}

// 	// Bail if pos data not new
// 	if (!is_ekfNew)
// 	{
		
// 		return isZoneTriggered;
// 	}

// 	// Reset flag
// 	is_ekfNew = false;

// 	// Check if all bounds passed
// 	if (RatPos > zoneBoundCumMax[zoneMax] + 5)
// 	{
// 		isAllZonePassed = true;
		
// 		return isZoneTriggered;
// 	}

// 	// Bail if first bound not reached
// 	if (RatPos < zoneBoundCumMin[zoneMin])
// 	{
		
// 		return isZoneTriggered;
// 	}

// 	// Check if rat in any bounds
// 	for (int i = zoneMin; i <= zoneMax; i++)
// 	{
// 		if (
// 			RatPos > zoneBoundCumMin[i] &&
// 			RatPos < zoneBoundCumMax[i]
// 			)
// 		{

// 			// Update timers
// 			t_lastZoneCheck = t_lastZoneCheck == 0 ? millis() : t_lastZoneCheck;
// 			t_nowZoneCheck = millis();

// 			// Store occupancy time
// 			zoneOccTim[i] += t_nowZoneCheck - t_lastZoneCheck;
// 			zoneOccCnt[i]++;
// 			t_lastZoneCheck = t_nowZoneCheck;

// 			// Check if occ thresh passed
// 			if (zoneOccTim[i] >= rewDelay)
// 			{

// 				// REWARD at this pos
// 				SetZoneDur(i);

// 				// Store reward info for debugging
// 				zoneRewarded = zoneLocs[i] * -1;
// 				zoneBoundCumRewarded[0] = zoneBoundCumMin[i];
// 				zoneBoundCumRewarded[1] = zoneBoundCumMax[i];
// 				occRewarded = zoneOccTim[i];

// 				// Set flag
// 				isZoneTriggered = true;
// 			}
// 		}
// 	}

	
// 	return isZoneTriggered;
// }



// void RewardReset(bool was_rewarded)
// {
	

// 	// Local vars

// 	// Log event
	

	
// 	// Reset flags etc
// 	isRewarding = false;
// 	isZoneTriggered = false;
// 	isAllZonePassed = false;
// 	is_ekfNew = false;

// 	// Reset occ time
// 	for (int i = 0; i < zoneLng; i++)
// 	{
// 		zoneOccTim[i] = 0;
// 		zoneOccCnt[i] = 0;
// 	}
// 	t_nowZoneCheck = 0;
// 	t_lastZoneCheck = 0;

	
// }