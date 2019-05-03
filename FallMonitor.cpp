	#include <FallMonitor.h>

	bool fDebug = false;
	
	/*#######################################
	  BEGIN FallMonitorConfigData name
	  #######################################**/
	uint16_t FallMonitorConfigData::FALL_IMPACT_SPAN = 3240;
	uint16_t FallMonitorConfigData::INSPECTION_MAX_MILLIS = 120;
	uint16_t FallMonitorConfigData::INSPECTION_MIN_MILLIS = 35;
	
	double FallMonitorConfigData::H_FALL_THRESH = 8.4d;
	double FallMonitorConfigData::M_FALL_THRESH = 6.4d;
	double FallMonitorConfigData::L_FALL_THRESH = 3.7d;
	
	double FallMonitorConfigData::L_IMPACT_THRESH = 13.0d;
	double FallMonitorConfigData::M_IMPACT_THRESH = 18.0d;
	double FallMonitorConfigData::H_IMPACT_THRESH = 25.0d;
	
	double FallMonitorConfigData::LOWER_LIMIT = 0.1d;//kluge - anything below this is meaningless and means sensor still initializing
	/*#######################################
	  END FallMonitorConfigData name
	  #######################################**/
	
	namespace fmcd = FallMonitorConfigData;
	
	/*#######################################
	  BEGIN FallMonitorContext name
	  #######################################**/
	bool FallMonitorContext::maybeFalling = false;
    bool FallMonitorContext::likelyFalling = false;
    
    bool FallMonitorContext::maybeImpacting = false;
    bool FallMonitorContext::likelyImpacting = false;
    
    bool FallMonitorContext::inspectionComplete = false;
    bool FallMonitorContext::movementReportable = false;
    int FallMonitorContext::movementType = NO_ANNOUNCE;

    double FallMonitorContext::aMag = 0.0d;
    
    unsigned long FallMonitorContext::tFallStart = 0L; 
    
    double FallMonitorContext::xVal = 0.0d;
    double FallMonitorContext::yVal = 0.0d;
    double FallMonitorContext::zVal = 0.0d;
    
    int FallMonitorContext::numExpiredChecks = 0;  
	
	void FallMonitorContext::setCoordVals(sensors_event_t* e){
		xVal = e->acceleration.x;
		yVal = e->acceleration.y;
		zVal = e->acceleration.z;
        aMag = sqrt(pow(xVal, 2.0) + pow(yVal, 2.0) + pow(zVal, 2.0));
    }
	
	/* should return true only if collection arrays have a great many enabled profiles
	 * remind: collections and profiles are reset when maybe falling goes from false to true*/
	void FallMonitorContext::checkInspectionComplete(){	
		inspectionComplete = uint16_t(millis() - tFallStart) > fmcd::FALL_IMPACT_SPAN; //(++numExpiredChecks > 150) && (uint16_t(millis() - tFallStart) > fmcd::FALL_IMPACT_SPAN);
	}
	
	/*  return true only if maybeImpacting is changed to true from false */
    bool FallMonitorContext::resetMaybeImpacting(){
    	return (!maybeImpacting && aMag > fmcd::L_IMPACT_THRESH) ? true : false;
    }
    
    /*  return true if maybeFalling is changing to true from false */
    bool FallMonitorContext::resetMaybeFalling(){
    	return (!maybeFalling && aMag < fmcd::H_FALL_THRESH) ? true : false;
    }
    
    double FallMonitorContext::aMagAverage(double aMagTotal, uint16_t count){
    	return aMagTotal/count;
    }
    
    int FallMonitorContext::getCategory(int type){
    	int retVal = NO_CAT;
    	switch(type){
			case FALL_WEAK_CRITERIA:
				retVal = FALLING_CAT;
				break;
				
			case FALL_STRONG_CRITERIA:
				retVal = FALLING_CAT;
				break;
				
			case IMPACT_WEAK_CRITERIA:
				retVal = IMPACTING_CAT;
				break;
				
			case IMPACT_STRONG_CRITERIA:
				retVal = IMPACTING_CAT;
				break;
				
			default:
				retVal = NO_CAT;
				break;
    	}
    	return retVal;
    }
    
    bool FallMonitorContext::doProcessType(int type){
    	bool retVal = false;
    	int cat = getCategory(type);
    	switch (cat){
			case FALLING_CAT:
				retVal = maybeFalling;
				break;
				
			case IMPACTING_CAT:
				retVal = maybeImpacting;
				break;
				
			default:
				break;
    	}
    	return retVal;
    }
	/*#######################################
	  END FallMonitorContext name
	  #######################################**/
	
    namespace fmc = FallMonitorContext;

	/*#######################################
	  BEGIN Profile impl
	  #######################################**/
    
    AmagProfile* AmagProfileCollection::INVALID_PROFILE;

	/*#######################################
	  END Profile impl
	  #######################################**/
    
    
	/*#######################################
	  BEGIN AmagProfile impl
	  #######################################**/
    void AmagProfile::setTDelta(unsigned long nowMillis){
    	_tDelta = uint16_t(nowMillis-fmc::tFallStart) - _tOffset;
    }

    void AmagProfile::setAmagAverage(int type, double total, uint16_t count){
		_updateCount = count;
		_aMagTotal = total;
		_aMagAverage = fmc::aMagAverage(total, count);
		if (fDebug) Serial.printf("\nAmagProfile::setAmagAverage() type=%d total=%s count=%d _aMagAverage=%s", type, String(total).c_str(), count, String(_aMagAverage).c_str());
    }
    
    void AmagProfile::setEnabled(){ 
    	_startVal = fmc::aMag;
    	_startAve = _aMagAverage;
    	_tStartElapsed = _tDelta + _tOffset;
    	_isEnabled = true; 
    }
    
    void AmagProfile::setFinal(){ 
    	_finalVal = fmc::aMag;
    	_finalAve = _aMagAverage;
    	_tFinalElapsed = _tDelta + _tOffset;
    	_isFinal = true; 
    }
    
    void AmagProfile::update(int type, unsigned long nowMillis){
		setAmagAverage(type, _aMagTotal+fmc::aMag, _updateCount+1);
		setTDelta(nowMillis);
    }
    
    void AmagProfile::reset(){
    	_tDelta = 0;
    	_finalAve = 0.0d;
        _startAve = 0.0d;
        _aMagTotal = 0.0d;
        _aMagAverage = 0.0d;
        _tFinalElapsed = 0;
        _tStartElapsed = 0;
        Profile::reset();
    }
	/*#######################################
	  END AmagProfile impl
	  #######################################**/
    
    
	/*#######################################
	  BEGIN ProfileCollection impl
	  #######################################**/
    
	AmagProfile* AmagProfileCollection::getProfileAt(int itemIndex){
		if (fDebug) Serial.printf("\nAmagProfileCollection::getProfileAt itemIndex=%d", itemIndex);
		if (itemIndex<0){ 
			return INVALID_PROFILE; //should never get here
		}else{
			return &(_aProfiles[itemIndex]);
		}
    }
	/*#######################################
	  BEGIN ProfileCollection impl
	  #######################################**/

	
	/*#######################################
	  BEGIN AmagProfileCollection impl
	  #######################################**/
	void AmagProfileCollection::setFinal(AmagProfile* aPro){
		aPro->setFinal();
		_lastFinalProfile = aPro;
		if (_nextNonFinalProfileIndex < ARR_SIZE-1) _nextNonFinalProfileIndex++;
		_nextNonFinalProfile = getProfileAt(_nextNonFinalProfileIndex);
		_nextNonFinalProfile->setTOffset(aPro->getTFinalElapsed());
	}
	
	void AmagProfileCollection::setEnabled(AmagProfile* aPro){
		aPro->setEnabled();
		_lastEnabledProfile = aPro;
	}
    
    void AmagProfileCollection::reset(){
    	if (fDebug) Serial.printf("\nAmagProfile::reset() BEGIN!! type=%d", _type);
    	for (int i=0; i<ARR_SIZE; i++){
    		AmagProfile* aPro = getProfileAt(i);
    		aPro->reset();
            //if (fDebug) Serial.printf("\nAmagProfileCollection::reset() int i=%d", i);
        }
    	_lastEnabledProfile = INVALID_PROFILE;
    	_lastFinalProfile = INVALID_PROFILE;
    	_nextNonFinalProfile = INVALID_PROFILE;
    	
    	_nextNonFinalProfileIndex = -1;
        if (fDebug) Serial.printf("\nAmagProfile::reset() END!! type=%d", _type);
    }

	AmagProfile* AmagProfileCollection::getLastEnabledProfile(){
		return _lastEnabledProfile;
	}
	
	AmagProfile* AmagProfileCollection::getLastFinalProfile(){
		return _lastFinalProfile;
	}
	
	AmagProfile* AmagProfileCollection::getNextNonFinalProfile(unsigned long nowMillis){	
		if (_nextNonFinalProfile == INVALID_PROFILE){
			_nextNonFinalProfileIndex = 0;
			_nextNonFinalProfile = getProfileAt(_nextNonFinalProfileIndex);
		}
		_nextNonFinalProfile->update(_type, nowMillis);
		return _nextNonFinalProfile;
	}

	AmagProfileCollection::AmagProfileCollection(){
		if (fDebug) Serial.printf("\nAmagProfileCollection::AmagProfileCollection() THIS SHOULD SHOW UP ONLY TWICE!!");
	}
	
	AmagProfileCollection::~AmagProfileCollection(){
	}
	/*#######################################
	  END AmagProfileCollection impl
	  #######################################**/

    
	/*#######################################
	  BEGIN AccelerationAnalyzer impl
	  #######################################**/
	AmagProfileCollection* AccelerationAnalyzer::getCollectionAt(int itemIndex){
		return &(_aProColls[itemIndex]);
    }
	
    AmagProfileCollection* AccelerationAnalyzer::getCollectionType(int type){
    	int index=0;
    	switch (type){
			case fmc::FALL_WEAK_CRITERIA:
				index=0;
				break;
			case fmc::FALL_STRONG_CRITERIA:
				index=1;
				break;
			case fmc::IMPACT_WEAK_CRITERIA:
				index=2;
				break;
			case fmc::IMPACT_STRONG_CRITERIA:
				index=3;
				break;
			default:
				break;
    	}
    	return getCollectionAt(index);
    }
	
    void AccelerationAnalyzer::resetAmagProfileCollections(){
    	if (fDebug) Serial.printf("\nTWO AmagProfileCollections::reset() plural about to BEGIN!!");
    	for (int i=0; i<NUM_PROFILE_COLL_TYPES; i++){
    		AmagProfileCollection* aProColl = getCollectionAt(i);
    		aProColl->reset();
        }
    }
    
    /*
     * return of false does NOT mean likely not falling. what is best/uncoupled implementation here??
     * note: this should be considered biz logic, meaning does not belong as a method of the collection even though it is highly collection type dependent
     */
    uint8_t AccelerationAnalyzer::getWorkflowStatus(uint16_t tDelta, double aMagAverage, int type){
    	bool inspectionExpired;
    	bool inspectionBegin;
    	bool lowLevelInspectionTimeout;
    	
    	if (fDebug) Serial.printf("\nAccelerationAnalyzer::getWorkflowStatus tDelta=%d aMagAverage=%s type=%d", tDelta, String(aMagAverage).c_str(), type);

        uint8_t retStatus = fmc::NO_STATUS;
        switch (type){
            case fmc::FALL_WEAK_CRITERIA:  	
            	//inspecting thresholds of medium acceleration that may indicate some sort of falling
            	inspectionExpired = tDelta >= fmcd::INSPECTION_MAX_MILLIS;
                if (inspectionExpired){
                    retStatus = fmc::INSPECTION_EXPIRED;

                }else{    	
                    inspectionBegin = tDelta >= fmcd::INSPECTION_MIN_MILLIS;
                    if (inspectionBegin){
						retStatus = (aMagAverage < fmcd::M_FALL_THRESH) ? fmc::BELOW_M_FALL_THRESH : fmc::ABOVE_M_FALL_THRESH;			
						//if (true) Serial.printf("\nWorkflowStatus M_FALL_THRESH check tDelta=%d aMagAverage=%s", tDelta, String(aMagAverage).c_str());
                    }else{
                    	retStatus = fmc::NO_STATUS_CHANGE;
                    }
                }
                break;
        
            case fmc::FALL_STRONG_CRITERIA:
            	//strictly inspecting thresholds of acceleration below and above near free fall
            	retStatus = (fmc::aMag < fmcd::L_FALL_THRESH) ? fmc::BELOW_L_FALL_THRESH : fmc::ABOVE_L_FALL_THRESH;
                break;

            case fmc::IMPACT_WEAK_CRITERIA:  	
            	//inspecting thresholds of medium acceleration that may indicate some sort of falling
            	inspectionExpired = tDelta >= fmcd::INSPECTION_MAX_MILLIS;
                if (inspectionExpired){
                    retStatus = fmc::INSPECTION_EXPIRED;

                }else{    	
                    inspectionBegin = tDelta >= fmcd::INSPECTION_MIN_MILLIS;
                    if (inspectionBegin){
						retStatus = (aMagAverage > fmcd::M_IMPACT_THRESH) ?  fmc::ABOVE_M_IMPACT_THRESH : fmc::BELOW_M_IMPACT_THRESH;
						if (fDebug) Serial.printf("\nWorkflowStatus M_IMPACT_THRESH check tDelta=%d aMagAverage=%s", tDelta, String(aMagAverage).c_str());
                    }else{
                    	retStatus = fmc::NO_STATUS_CHANGE;
                    }
                }
                break;
        
            case fmc::IMPACT_STRONG_CRITERIA:
            	retStatus = (fmc::aMag > fmcd::H_IMPACT_THRESH) ? fmc::ABOVE_H_IMPACT_THRESH : fmc::BELOW_H_IMPACT_THRESH;
                break;
                
           	default:
                break;
        }
        return retStatus;
    }
    
    void AccelerationAnalyzer::processAcceleration(unsigned long nowMillis){	
    	if (fDebug) Serial.printf("\nAccelerationAnalyzer::processAcceleration 1");

		for (int type=1; type<=NUM_PROFILE_COLL_TYPES; type++){
			
			if (!fmc::doProcessType(type)) return;
			
			AmagProfileCollection* aProColl = getCollectionType(type);
			AmagProfile* aProNNF = aProColl->getNextNonFinalProfile(nowMillis);
			if (fDebug) Serial.printf("\nAccelerationAnalyzer::processAcceleration 4 type=%d", type);
		
			uint8_t workflowStatus = (aProNNF != AmagProfileCollection::INVALID_PROFILE) ?  getWorkflowStatus(aProNNF->getTDelta(), aProNNF->getAmagAverage(), aProColl->getType()) : fmc::ERROR;

			switch(workflowStatus){
			
				case fmc::BELOW_M_FALL_THRESH:
					if (fmc::maybeFalling && !fmc::likelyFalling) {
						fmc::likelyFalling = true;
						if (fDebug) Serial.printf("\nAccelerationAnalyzer::processAcceleration BELOW_M_FALL_THRESH!! fmc::likelyFalling set to true!!");
					}
					/*collection has served up the next profile (meaning not set as final yet). after fmc::INSPECTION_MIN_MILLIS has passed, inspect the 
					 * AVERAGE acceleration (i.e. aMagAverage) - if it is BELOW a certain value then some sort of falling is happening. 
					 * mark this next profile as enabled at this point and for that reason. wait for the duration of the inspection period 
					 * which is fmc::INSPECTION_MAX_MILLIS before marking this profile as FINAL which indicates that it's time for the collection to
					 * serve up the profile that comes next in the array*/
					if (!aProNNF->isEnabled()) aProColl->setEnabled(aProNNF);
					break;
				
				case fmc::ABOVE_M_FALL_THRESH:
					/*collection has served up an enabled profile (meaning there was some sort of a falling event detected). the time span of 
					 * fmc::INSPECTION_MAX_MILLIS has not yet passed for this profile. instead, before it passes the falling
					 * event may have changed to not likely falling, at least for the current time interval of fmc::INSPECTION_MAX_MILLIS*/
					if (aProNNF->isEnabled()) aProColl->setFinal(aProNNF);
					break;
					
				case fmc::ABOVE_M_IMPACT_THRESH:
					if (fmc::maybeImpacting && !fmc::likelyImpacting) {
						fmc::likelyImpacting = true;
						if (fDebug) Serial.printf("\nAccelerationAnalyzer::processAcceleration ABOVE_M_IMPACT_THRESH!! fmc::likelyImpacting set to true!!");
					}
					/*collection has served up the next profile (meaning not set as final yet). after fmc::INSPECTION_MIN_MILLIS has passed, inspect the 
					 * AVERAGE acceleration (i.e. aMagAverage) - if it is ABOVE a certain value then some sort of impact is happening. 
					 * mark this next profile as enabled at this point and for that reason. wait for the duration of the inspection period 
					 * which is fmc::INSPECTION_MAX_MILLIS before marking this profile as FINAL which indicates that it's time for the collection to
					 * serve up the profile that comes next in the array*/
					if (!aProNNF->isEnabled()) aProColl->setEnabled(aProNNF);
					break;
				
				case fmc::BELOW_M_IMPACT_THRESH:
					/*collection has served up an enabled profile (meaning there was some sort of a impacting event detected). the time span of 
					 * fmc::INSPECTION_MAX_MILLIS has not yet passed for this profile. instead, before it passes the impacting
					 * event may have changed to not likely impacting, at least for the current time interval of fmc::INSPECTION_MAX_MILLIS*/
					if (aProNNF->isEnabled()) aProColl->setFinal(aProNNF);
					break;	

				case fmc::BELOW_L_FALL_THRESH:
					/*set profile start values the moment a near free fall occurs*/
					if (!aProNNF->isEnabled()) aProColl->setEnabled(aProNNF);
					break;
					
				case fmc::ABOVE_L_FALL_THRESH:
					/*set profile final values the moment a near free fall ends - note there is no inspection expiration for this collection type*/
					if (aProNNF->isEnabled()) aProColl->setFinal(aProNNF);
					break;
					
				case fmc::ABOVE_H_IMPACT_THRESH:
					/*set profile start values the moment a big impact occurs*/
					if (!aProNNF->isEnabled()) aProColl->setEnabled(aProNNF);
					break;
					
				case fmc::BELOW_H_IMPACT_THRESH:
					/*set profile final values the moment a big impact ends - note there is no inspection expiration for this collection type*/
					if (aProNNF->isEnabled()) aProColl->setFinal(aProNNF);
					break;
					
				case fmc::INSPECTION_EXPIRED:
					if (fmc::maybeFalling && !fmc::likelyFalling){
						fmc::maybeFalling = false;
						if (fDebug) Serial.printf("\nAccelerationAnalyzer::processAcceleration INSPECTION_EXPIRED!! maybeFalling=false");
						
					}
					if (fmc::maybeImpacting && !fmc::likelyImpacting){
						fmc::maybeImpacting = false;
						if (fDebug) Serial.printf("\nAccelerationAnalyzer::processAcceleration INSPECTION_EXPIRED!! maybeImpacting=false");
						
					}
					if (fmc::likelyFalling || fmc::likelyImpacting){	
						//something is still happening so ... set em
						if (!aProNNF->isEnabled()) aProColl->setEnabled(aProNNF);//in this case falling or impacting criteria were never met all through inspection to the end
						if (aProNNF->isEnabled()) aProColl->setFinal(aProNNF);
					}
					break;
					
				case fmc::NO_STATUS_CHANGE:
					break;
					
				case fmc::NO_STATUS:
					if (fDebug) Serial.printf("\nAccelerationAnalyzer::processAcceleration NO_STATUS!! should never happen");
					break;
					
				case fmc::ERROR:
					if (fDebug) Serial.printf("\nAccelerationAnalyzer::processAcceleration ERROR!! check collection indices");
					break;
				
				default:
					break;
			}
		}
    }
    
    void AccelerationAnalyzer::analyzeCollections(){
    	/*
    	 * 1. figure out how to upload via blynk
    	 * 2. only after full display then reset collections/profiles/fallmanager
    	 * 		a. how to do the above if uploading to Blynk means once a second? can you analyze elsewhere?
    	 * 		b. does it do any good to show graphical analysis via blynk when blynk enthusiasts know how slow it was?
    	 */
    	
		double aMagAve = 0.0d;
		double aMagInst = 0.0d;
		uint16_t aMagTSpan = 0;
		uint16_t aMagT = 0;
    	
        int i=0;
        if (true) Serial.printf("\n\nAccelerationAnalyzer::analyzeCollections WEAK FALL CRITERIA ... \n\n");
        
        while (i < ProfileCollection::ARR_SIZE){
            AmagProfile* aPro = getCollectionAt(0)->getProfileAt(i);
            i++;
            if (aPro->isFinal()) {
            	if (true) Serial.printf("\nAccelerationAnalyzer::analyzeCollections index=%d startTime=%d & startAve=%s finalTime=%d & finalAve=%s tOffset=%d tDelta=%d", i, aPro->getTStartElapsed(), String(aPro->getStartAve()).c_str(), aPro->getTFinalElapsed(), String(aPro->getFinalAve()).c_str(), aPro->getTOffset(), aPro->getTDelta());
                //Blynk.virtualWrite(V101, aPro->getFinalAve());
                //delay(1200L);
                //yield();
            	//fmc::movementType = fmc::SITTING_ANNOUNCE;
            	
            	aMagAve = (aPro->getStartAve() + aPro->getFinalAve())/2;
            	aMagTSpan = aPro->getTFinalElapsed() - aPro->getTStartElapsed();
            	aMagT = aPro->getTOffset() + aPro->getTDelta();
            	
            	if (i==0) {
            		lowAmag1 = lowAmag2 = lowAmag3 = lowAmag4 = aMagAve;
            		lowAmag1T = lowAmag2T = lowAmag3T = lowAmag4T = aMagT;
            		lowAmag1TSpan = lowAmag2TSpan = lowAmag3TSpan = lowAmag4TSpan = aMagTSpan;
            	}else{
            		
					if (aMagAve < lowAmag1) {
						lowAmag4 = lowAmag3;
						lowAmag4TSpan = lowAmag3TSpan;
						lowAmag4T = lowAmag3T;
						
						lowAmag3 = lowAmag2;
						lowAmag3TSpan = lowAmag2TSpan;
						lowAmag3T = lowAmag2T;
						
						lowAmag2 = lowAmag1;
						lowAmag2TSpan = lowAmag1TSpan;
						lowAmag2T = lowAmag1T;
						
						lowAmag1 = aMagAve;
						lowAmag1TSpan = aMagTSpan;
						lowAmag1T = aMagT;
						
					}else if (aMagAve < lowAmag2) {
						lowAmag4 = lowAmag3;
						lowAmag4TSpan = lowAmag3TSpan;
						lowAmag4T = lowAmag3T;
						
						lowAmag3 = lowAmag2;
						lowAmag3TSpan = lowAmag2TSpan;
						lowAmag3T = lowAmag2T;
						
						lowAmag2 = aMagAve;
						lowAmag2TSpan = aMagTSpan;
						lowAmag2T = aMagT;	
						
					}else if (aMagAve < lowAmag3) {
						lowAmag4 = lowAmag3;
						lowAmag4TSpan = lowAmag3TSpan;
						lowAmag4T = lowAmag3T;
						
						lowAmag3 = aMagAve;
						lowAmag3TSpan = aMagTSpan;
						lowAmag3T = aMagT;
						
					}else if (aMagAve < lowAmag4) {
						lowAmag4 = aMagAve;
						lowAmag4TSpan = aMagTSpan;
						lowAmag4T = aMagT;
						
					}
            	}
            
            }else{
            	break;
            }
        }
      
        if (lowAmag1 < fmcd::M_FALL_THRESH && lowAmag1TSpan > 40 && lowAmag2 < fmcd::M_FALL_THRESH && lowAmag2TSpan > 40){
        	fmc::movementType = fmc::FALLING_WEAK_ANNOUNCE;
        }
        
        if (true) Serial.printf("\n\nAccelerationAnalyzer::analyzeCollections STRONG FALL CRITERIA ... \n\n");


        i=0;
        while (i < ProfileCollection::ARR_SIZE){
            AmagProfile* aPro = getCollectionAt(1)->getProfileAt(i);
            i++;		
            if (aPro->isEnabled()) {
            	if (true) Serial.printf("\nAccelerationAnalyzer::analyzeCollections index=%d startTime=%d & startVal=%s finalTime=%d & finalVal=%s tOffset=%d tDelta=%d", i, aPro->getTStartElapsed(), String(aPro->getStartVal()).c_str(), aPro->getTFinalElapsed(), String(aPro->getFinalVal()).c_str(), aPro->getTOffset(), aPro->getTDelta());
                //Blynk.virtualWrite(V101, aPro->getFinalAve());
                //delay(1200L);
                //yield();
            	//fmc::movementType = fmc::FALLING_WEAK_ANNOUNCE;
            	
            	aMagInst = aPro->getStartVal();
            	aMagTSpan = aPro->getTFinalElapsed() - aPro->getTStartElapsed();
            	aMagT = aPro->getTOffset();

            	if (i==0) {
            		lowAmag6 = lowAmag7  = lowAmag8 = aMagInst;
            		lowAmag6T = lowAmag7T = lowAmag8T = aMagT;
            		lowAmag6TSpan = lowAmag7TSpan = lowAmag8TSpan = aMagTSpan;
            	}else{
            		
					if (aMagInst < lowAmag6) {
						lowAmag8 = lowAmag7;
						lowAmag8TSpan = lowAmag7TSpan;
						lowAmag8T = lowAmag7T;
						
						lowAmag7 = lowAmag6;
						lowAmag7TSpan = lowAmag6TSpan;
						lowAmag7T = lowAmag6T;
						
						lowAmag6 = aMagInst;
						lowAmag6TSpan = aMagTSpan;
						lowAmag6T = aMagT;	
						
					}else if (aMagInst < lowAmag7) {
						lowAmag8 = lowAmag7;
						lowAmag8TSpan = lowAmag7TSpan;
						lowAmag8T = lowAmag7T;
						
						lowAmag7 = aMagInst;
						lowAmag7TSpan = aMagTSpan;
						lowAmag7T = aMagT;
						
					}else if (aMagInst < lowAmag8) {
						lowAmag8 = aMagInst;
						lowAmag8TSpan = aMagTSpan;
						lowAmag8T = aMagT;
						
					}
            	}
            
            }else{
            	break;
            }

        }
        
        if (lowAmag6 < fmcd::L_FALL_THRESH && lowAmag6TSpan > 90){
        	fmc::movementType = fmc::FALLING_WEAK_ANNOUNCE;
        }
        if (lowAmag6 < fmcd::L_FALL_THRESH && lowAmag6TSpan > 160){
        	fmc::movementType = fmc::FALLING_STRONG_ANNOUNCE;
        }
        
        if (true) Serial.printf("\n\nAccelerationAnalyzer::analyzeCollections WEAK IMPACT CRITERIA ... \n\n");

        i=0;
        while (i < ProfileCollection::ARR_SIZE){
            AmagProfile* aPro = getCollectionAt(2)->getProfileAt(i);
            i++;		
            if (aPro->isEnabled()) {
            	if (true) Serial.printf("\nAccelerationAnalyzer::analyzeCollections index=%d startTime=%d & startVal=%s finalTime=%d & finalVal=%s tOffset=%d tDelta=%d", i, aPro->getTStartElapsed(), String(aPro->getStartVal()).c_str(), aPro->getTFinalElapsed(), String(aPro->getFinalVal()).c_str(), aPro->getTOffset(), aPro->getTDelta());
                //Blynk.virtualWrite(V121, aPro->getFinalAve());
                //delay(1200L);
                //yield();
            	//fmc::movementType = fmc::SITTING_ANNOUNCE;
            	
            	aMagAve = (aPro->getStartAve() + aPro->getFinalAve())/2;
            	aMagTSpan = aPro->getTFinalElapsed() - aPro->getTStartElapsed();
            	aMagT = aPro->getTOffset() + aPro->getTDelta();
            	
            	if (i==0) {
            		highAmag1 = highAmag2 = highAmag3 = highAmag4 = aMagAve;
            		highAmag1T = highAmag2T = highAmag3T = highAmag4T = aMagT;
            		highAmag1TSpan = highAmag2TSpan = highAmag3TSpan = highAmag4TSpan = aMagTSpan;
            	}else{
            		
					if (aMagAve > highAmag1) {
						highAmag4 = highAmag3;
						highAmag4TSpan = highAmag3TSpan;
						highAmag4T = highAmag3T;
						
						highAmag3 = highAmag2;
						highAmag3TSpan = highAmag2TSpan;
						highAmag3T = highAmag2T;
						
						highAmag2 = highAmag1;
						highAmag2TSpan = highAmag1TSpan;
						highAmag2T = highAmag1T;
						
						highAmag1 = aMagAve;
						highAmag1TSpan = aMagTSpan;
						highAmag1T = aMagT;
						
					}else if (aMagAve > highAmag2) {
						highAmag4 = highAmag3;
						highAmag4TSpan = highAmag3TSpan;
						highAmag4T = highAmag3T;
						
						highAmag3 = highAmag2;
						highAmag3TSpan = highAmag2TSpan;
						highAmag3T = highAmag2T;
						
						highAmag2 = aMagAve;
						highAmag2TSpan = aMagTSpan;
						highAmag2T = aMagT;	
						
					}else if (aMagAve > highAmag3) {
						highAmag4 = highAmag3;
						highAmag4TSpan = highAmag3TSpan;
						highAmag4T = highAmag3T;
						
						highAmag3 = aMagAve;
						highAmag3TSpan = aMagTSpan;
						highAmag3T = aMagT;
						
					}else if (aMagAve > highAmag4) {
						highAmag4 = aMagAve;
						highAmag4TSpan = aMagTSpan;
						highAmag4T = aMagT;
						
					}
            	}
            
            }else{
            	break;
            }

        }
        
        if (fmc::movementType != fmc::FALLING_STRONG_ANNOUNCE){
			if (highAmag3 > fmcd::M_IMPACT_THRESH && highAmag3TSpan > 40 && highAmag4 > fmcd::M_IMPACT_THRESH && highAmag4TSpan > 40){
				fmc::movementType = fmc::FALLING_WEAK_ANNOUNCE;
			}
        }

        if (true) Serial.printf("\n\nAccelerationAnalyzer::analyzeCollections STRONG IMPACT CRITERIA ... \n\n");

        i=0;
        while (i < ProfileCollection::ARR_SIZE){
            AmagProfile* aPro = getCollectionAt(3)->getProfileAt(i);
            i++;		
            if (aPro->isEnabled()) {
            	if (true) Serial.printf("\nAccelerationAnalyzer::analyzeCollections index=%d startTime=%d & startVal=%s finalTime=%d & finalVal=%s tOffset=%d tDelta=%d", i, aPro->getTStartElapsed(), String(aPro->getStartVal()).c_str(), aPro->getTFinalElapsed(), String(aPro->getFinalVal()).c_str(), aPro->getTOffset(), aPro->getTDelta());
                //Blynk.virtualWrite(V101, aPro->getFinalAve());
                //delay(1200L);
                //yield();
            	//fmc::movementType = fmc::FALLING_WEAK_ANNOUNCE;
            	
            	aMagInst = aPro->getStartVal();
            	aMagTSpan = aPro->getTFinalElapsed() - aPro->getTStartElapsed();
            	aMagT = aPro->getTOffset();

            	if (i==0) {
            		highAmag6 = highAmag7  = highAmag8 = aMagInst;
            		highAmag6T = highAmag7T = highAmag8T = aMagT;
            		highAmag6TSpan = highAmag7TSpan = highAmag8TSpan = aMagTSpan;
            	}else{
            		
					if (aMagInst > highAmag6) {
						highAmag8 = highAmag7;
						highAmag8TSpan = highAmag7TSpan;
						highAmag8T = highAmag7T;
						
						highAmag7 = highAmag6;
						highAmag7TSpan = highAmag6TSpan;
						highAmag7T = highAmag6T;
						
						highAmag6 = aMagInst;
						highAmag6TSpan = aMagTSpan;
						highAmag6T = aMagT;	
						
					}else if (aMagInst > highAmag7) {
						highAmag8 = highAmag7;
						highAmag8TSpan = highAmag7TSpan;
						highAmag8T = highAmag7T;
						
						highAmag7 = aMagInst;
						highAmag7TSpan = aMagTSpan;
						highAmag7T = aMagT;
						
					}else if (aMagInst > highAmag8) {
						highAmag8 = aMagInst;
						highAmag8TSpan = aMagTSpan;
						highAmag8T = aMagT;
						
					}
            	}
            }else{
            	break;
            }

        }
        
        if (fmc::movementType != fmc::FALLING_STRONG_ANNOUNCE){
			if (highAmag7 > fmcd::H_IMPACT_THRESH && highAmag7TSpan > 50){
				fmc::movementType = fmc::FALLING_WEAK_ANNOUNCE;
			}
			if (highAmag7 > fmcd::H_IMPACT_THRESH && highAmag7TSpan > 90){
				fmc::movementType = fmc::FALLING_STRONG_ANNOUNCE;
			}
        }
        
        if (fmc::movementType == fmc::NO_ANNOUNCE){
        	fmc::movementReportable = false;
        }else{
        	fmc::movementReportable = true;
        }
		
    }  
    
    void AccelerationAnalyzer::resetReport(){
    	lowAmag1 = 100.0d;
    	lowAmag2 = 100.0d;
    	lowAmag3 = 100.0d;
    	lowAmag4 = 100.0d;
    	lowAmag6 = 100.0d;
    	lowAmag7 = 100.0d;
    	lowAmag8 = 100.0d;
    	    	
    	highAmag1 = 0.0d;
    	highAmag2 = 0.0d;
    	highAmag3 = 0.0d;
    	highAmag4 = 0.0d;
    	highAmag6 = 0.0d;
    	highAmag7 = 0.0d;
    	highAmag8 = 0.0d;
    	
    	lowAmag1T = 0;
    	lowAmag2T = 0;
    	lowAmag3T = 0;
    	lowAmag4T = 0;
    	lowAmag6T = 0;
    	lowAmag7T = 0;
    	lowAmag8T = 0;
    	
    	highAmag1T = 0;
    	highAmag2T = 0;
    	highAmag3T = 0;
    	highAmag4T = 0;
    	highAmag6T = 0;
    	highAmag7T = 0;
    	highAmag8T = 0;
    	
    	lowAmag1TSpan = 0;
    	lowAmag2TSpan = 0;
    	lowAmag3TSpan = 0;
    	lowAmag4TSpan = 0;
    	lowAmag6TSpan = 0;
    	lowAmag7TSpan = 0;
    	lowAmag8TSpan = 0;
    	
    	highAmag1TSpan = 0;
    	highAmag2TSpan = 0;
    	highAmag3TSpan = 0;
    	highAmag4TSpan = 0;
    	highAmag6TSpan = 0;
    	highAmag7TSpan = 0;	
    	highAmag8TSpan = 0;	
    }
    
    AccelerationAnalyzer::AccelerationAnalyzer(){
    	if (fDebug) Serial.printf("\nAccelerationAnalyzer::AccelerationAnalyzer created=%s", "true");

    	for (int i=0; i< NUM_PROFILE_COLL_TYPES; i++){
    		int type = i+1;
    		AmagProfileCollection* aProColl = getCollectionAt(i);
    		aProColl->setType(type);
    	}
    	
    	for (int i=0; i< NUM_PROFILE_COLL_TYPES; i++){
			Serial.printf("\nAccelerationAnalyzer::AccelerationAnalyzer i=%d & type=%d ", i, _aProColls[i].getType());
    	}
    }
    
    AccelerationAnalyzer::~AccelerationAnalyzer(){
    }
	/*#######################################
	  END AccelerationAnalyzer impl
	  #######################################**/
	
	
	/*#######################################
	  BEGIN FallManager impl
	  #######################################**/	
	void FallManager::reset(){
		if (true) Serial.printf("\nFallManager::reset() BEGIN!!");
		fmc::maybeFalling = false;
    	fmc::likelyFalling = false;
    	
		fmc::maybeImpacting = false;
		fmc::likelyImpacting = false;

		fmc::inspectionComplete = false;
		fmc::movementType = fmc::NO_ANNOUNCE;
		fmc::movementReportable = false;
		
		fmc::numExpiredChecks = 0;
    	fmc::tFallStart = millis();
        aa.resetAmagProfileCollections();
        if (true) Serial.printf("\nFallManager::reset() FINISH!!");
    }
    
    int FallManager::getMovementType(){
    	return fmc::movementType;
    }
    
    bool FallManager::isInspectionComplete(){
    	fmc::checkInspectionComplete();
    	return fmc::inspectionComplete;
    }
    
    bool FallManager::isLikelyFalling(){
        return fmc::likelyFalling;
    }

    bool FallManager::isLikelyImpacting(){
        return fmc::likelyImpacting;
    }

    bool FallManager::isMaybeFallingOrImpacting(){
    	// maybeFallingReset and maybeImpactingReset never true simultaneously
    	// only way either can be true is if maybeFalling|maybeImpacting begin as false
		// if either maybeFalling or maybeImpacting is true then do not reset!!
    	if (fmc::resetMaybeFalling()){
    		if (!fmc::maybeImpacting) reset();
    		fmc::maybeFalling = true;
    		
    	}
    	if (fmc::resetMaybeImpacting()){
    		if (!fmc::maybeFalling) reset();
    		fmc::maybeImpacting = true;
    	}

    	if (fmc::maybeImpacting || fmc::maybeFalling){
    		//Serial.printf("\nFallManager::isMaybeFallingOrImpacting fmc::maybeImpacting=%s fmc::maybeFalling=%s", fmc::maybeImpacting?"true":"false", fmc::maybeFalling?"true":"false");
    	}
    	
    	return (fmc::maybeFalling || fmc::maybeImpacting);
    }
    
    void FallManager::processAcceleration(unsigned long nowMillis){
    	/* workhorse: run the span of fmcd::FALL_IMPACT_SPAN and populate arrays as long as fmc::likelyFalling is true */
    	aa.processAcceleration(nowMillis);
    }
    
    bool FallManager::setAmag(sensors_event_t* e){
    	fmc::setCoordVals(e);
    	if (fmc::aMag < fmcd::LOWER_LIMIT){
    		//Serial.printf("\nFallManager::isMaybeFalling ACCELERATION IS TOO LOW - MEANINGLESS EVEN, SO STOP WORKFLOW NOW	!!!");
    		return false;
    	}else{
    		return true;
    	}
    }

    void FallManager::reportToBlynk(){
    	Blynk.run();
    	Blynk.virtualWrite(V20, "clr");
		Blynk.virtualWrite(V21, "clr");Blynk.run();
		Blynk.virtualWrite(V22, "clr");Blynk.run();
		Blynk.virtualWrite(V23, "clr");Blynk.run();
		
    	Blynk.virtualWrite(V20, "add", 0, " Falls", " ");Blynk.run();
    	Blynk.virtualWrite(V21, "add", 0, " Span (ms)", "Start ");Blynk.run();
    	Blynk.virtualWrite(V20, "add", 1, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V21, "add", 1, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V20, "add", 2, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V21, "add", 2, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V20, "add", 3, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V21, "add", 3, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V20, "add", 4, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V21, "add", 4, " ", " ");Blynk.run();
    	
    	Blynk.virtualWrite(V22, "add", 0, " Impacts", " ");Blynk.run();
    	Blynk.virtualWrite(V23, "add", 0, " Span (ms)", "Start ");Blynk.run();
    	Blynk.virtualWrite(V22, "add", 1, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V23, "add", 1, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V22, "add", 2, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V23, "add", 2, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V22, "add", 3, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V23, "add", 3, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V22, "add", 4, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V23, "add", 4, " ", " ");Blynk.run();

    	Blynk.virtualWrite(V20, "update", 1, String(String("     ")+String(aa.lowAmag1)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V21, "update", 1, String(String("    ")+String(aa.lowAmag1TSpan)).c_str(), String(aa.lowAmag1T).c_str());Blynk.run();
    	Blynk.virtualWrite(V20, "update", 2, String(String("     ")+String(aa.lowAmag2)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V21, "update", 2, String(String("    ")+String(aa.lowAmag2TSpan)).c_str(), String(aa.lowAmag2T).c_str());Blynk.run();
    	Blynk.virtualWrite(V20, "update", 3, String(String("     ")+String(aa.lowAmag3)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V21, "update", 3, String(String("    ")+String(aa.lowAmag3TSpan)).c_str(), String(aa.lowAmag3T).c_str());Blynk.run();
    	Blynk.virtualWrite(V20, "update", 4, String(String("     ")+String(aa.lowAmag4)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V21, "update", 4, String(String("    ")+String(aa.lowAmag4TSpan)).c_str(), String(aa.lowAmag4T).c_str());Blynk.run();
        
    	Blynk.virtualWrite(V22, "update", 1, String(String("     ")+String(aa.highAmag1)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V23, "update", 1, String(String("    ")+String(aa.highAmag1TSpan)).c_str(), String(aa.highAmag1T).c_str());Blynk.run();
    	Blynk.virtualWrite(V22, "update", 2, String(String("     ")+String(aa.highAmag2)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V23, "update", 2, String(String("    ")+String(aa.highAmag2TSpan)).c_str(), String(aa.highAmag2T).c_str());Blynk.run();
    	Blynk.virtualWrite(V22, "update", 3, String(String("     ")+String(aa.highAmag3)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V23, "update", 3, String(String("    ")+String(aa.highAmag3TSpan)).c_str(), String(aa.highAmag3T).c_str());Blynk.run();
    	Blynk.virtualWrite(V22, "update", 4, String(String("     ")+String(aa.highAmag4)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V23, "update", 4, String(String("    ")+String(aa.highAmag4TSpan)).c_str(), String(aa.lowAmag4T).c_str());Blynk.run();
    	
    	Blynk.run();
    	Blynk.virtualWrite(V24, "clr");
		Blynk.virtualWrite(V25, "clr");Blynk.run();
		Blynk.virtualWrite(V26, "clr");Blynk.run();
		Blynk.virtualWrite(V27, "clr");Blynk.run();
		
    	Blynk.virtualWrite(V24, "add", 0, " Falls", " ");Blynk.run();
    	Blynk.virtualWrite(V25, "add", 0, " Span (ms)", "Start ");Blynk.run();
    	Blynk.virtualWrite(V24, "add", 1, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V25, "add", 1, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V24, "add", 2, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V25, "add", 2, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V24, "add", 3, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V25, "add", 3, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V24, "add", 4, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V25, "add", 4, " ", " ");Blynk.run();
    	
    	Blynk.virtualWrite(V26, "add", 0, " Impacts", " ");Blynk.run();
    	Blynk.virtualWrite(V27, "add", 0, " Span (ms)", "Start ");Blynk.run();
    	Blynk.virtualWrite(V26, "add", 1, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V27, "add", 1, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V26, "add", 2, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V27, "add", 2, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V26, "add", 3, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V27, "add", 3, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V26, "add", 4, " ", " ");Blynk.run();
    	Blynk.virtualWrite(V27, "add", 4, " ", " ");Blynk.run();

    	Blynk.virtualWrite(V24, "update", 1, String(String("     ")+String(aa.lowAmag6)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V25, "update", 1, String(String("    ")+String(aa.lowAmag6TSpan)).c_str(), String(aa.lowAmag6T).c_str());Blynk.run();
    	Blynk.virtualWrite(V24, "update", 2, String(String("     ")+String(aa.lowAmag7)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V25, "update", 2, String(String("    ")+String(aa.lowAmag7TSpan)).c_str(), String(aa.lowAmag7T).c_str());Blynk.run();
    	Blynk.virtualWrite(V24, "update", 3, String(String("     ")+String(aa.lowAmag8)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V25, "update", 3, String(String("    ")+String(aa.lowAmag8TSpan)).c_str(), String(aa.lowAmag8T).c_str());Blynk.run();
    	//Blynk.virtualWrite(V24, "update", 4, String(String("     ")+String(aa.lowAmag4)).c_str(), " ");Blynk.run();
    	//Blynk.virtualWrite(V25, "update", 4, String(String("    ")+String(aa.lowAmag4TSpan)).c_str(), String(aa.lowAmag4T).c_str());Blynk.run();
        
    	Blynk.virtualWrite(V26, "update", 1, String(String("     ")+String(aa.highAmag6)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V27, "update", 1, String(String("    ")+String(aa.highAmag6TSpan)).c_str(), String(aa.highAmag6T).c_str());Blynk.run();
    	Blynk.virtualWrite(V26, "update", 2, String(String("     ")+String(aa.highAmag7)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V27, "update", 2, String(String("    ")+String(aa.highAmag7TSpan)).c_str(), String(aa.highAmag7T).c_str());Blynk.run();
    	Blynk.virtualWrite(V26, "update", 3, String(String("     ")+String(aa.highAmag8)).c_str(), " ");Blynk.run();
    	Blynk.virtualWrite(V27, "update", 3, String(String("    ")+String(aa.highAmag8TSpan)).c_str(), String(aa.highAmag8T).c_str());Blynk.run();
    	//Blynk.virtualWrite(V26, "update", 4, String(String("     ")+String(aa.highAmag4)).c_str(), " ");Blynk.run();
    	//Blynk.virtualWrite(V27, "update", 4, String(String("    ")+String(aa.highAmag4TSpan)).c_str(), String(aa.lowAmag4T).c_str());Blynk.run();
    	
    	aa.resetReport();
    }
    
    bool FallManager::doAnalyzeAndReport(sensors_event_t* e, unsigned long nowMillis){
    	//if (true) Serial.printf("\ndoAnalyzeAndReport millis()-nowMillis = %u", millis()-nowMillis);
    		
    	/* calc and set magnitude of the current acceleration. return if invalid  */
		if (!setAmag(e)) return false;
		
		/* calc current amag - don't waste time if either maybeFalling|maybeImpacting is true or amag is too large|small */
		if (isMaybeFallingOrImpacting()){
			
			/* fmc::maybeFalling or fmc::maybeImpacting is true.
			 * do a super-short test to ensure integrity of maybeFalling|maybeImpacting determination. As short as 20ms. then ...
			 * 1. calc first amag average to determine if going from maybeFalling|maybeImpacting to likelyFalling|likelyImpacting
			 * 2. if moving to likelyFalling|likelyImpacting then store amagAverage in first array element(s)*/
			processAcceleration(nowMillis);
			if (isLikelyFalling() || isLikelyImpacting()){
			 
				/* fmc::likelyFalling or fmc::likelyImpacting is true.
				 * 1. determine that maybeFalling|maybeImpacting and likelyFalling|likelyImpacting should remain true. As long as 150ms. then ...
				 * 2. if #1 passes commit to execution over the full time span set via app FALL_IMPACT_SPAN
				 *      a. populate all arrays with AmagProfile elements
				 *      b. check often for inspection expiration */
				if (isInspectionComplete()){		
					/* analyze array collections to determine category: falling? sitting? walking? impact? nothing of interest? */
					aa.analyzeCollections();
					
					/*commented out above to check on delay it causes*/
					//aa.testAnalyzeCollections();
					
					/* are we certain it's safe to set maybeFalling to false here?? if not, everything can be wiped out
					 * it MUST be followed immediately by isFalling procedures which must complete before the next acceleration check
					 * in other words, before the next call to resetMaybeFallingImpacting()*/
					fmc::maybeFalling = false;
					fmc::maybeImpacting = false;
				}
			}
		}
		return fmc::movementReportable;
    }
    
    FallManager& FallManager::getInstance(){
         static FallManager instance;
         return instance;
     }
    
    
    FallManager::FallManager(){}
	/*#######################################
	  END FallManager impl
	  #######################################**/