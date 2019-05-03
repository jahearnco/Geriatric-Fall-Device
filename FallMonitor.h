#include <MyBlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>

/*#######################################
  BEGIN FallMonitorConfigData name
  #######################################**/
namespace FallMonitorConfigData{
	extern uint16_t FALL_IMPACT_SPAN;
	extern uint16_t INSPECTION_MAX_MILLIS;
	extern uint16_t INSPECTION_MIN_MILLIS;
	
	extern double H_FALL_THRESH;
	extern double M_FALL_THRESH;
	extern double L_FALL_THRESH;
	
	extern double L_IMPACT_THRESH;
	extern double M_IMPACT_THRESH;
	extern double H_IMPACT_THRESH;
	
	extern double LOWER_LIMIT;
}
/*#######################################
  END FallMonitorConfigData name
  #######################################**/



/*#######################################
  BEGIN FallMonitorContext name
  #######################################**/
namespace FallMonitorContext{
	//Fall Status consts
	const int FALL_WEAK_CRITERIA = 1;
	const int FALL_STRONG_CRITERIA = 2;
	const int IMPACT_WEAK_CRITERIA = 3;
	const int IMPACT_STRONG_CRITERIA = 4;
	
	const int NO_ANNOUNCE = 0;
	const int FALLING_STRONG_ANNOUNCE = 1;
	const int FALLING_WEAK_ANNOUNCE = 2;
	const int SITTING_ANNOUNCE = 3;
	const int WALKING_ANNOUNCE = 4;
	const int IMPACT_STRONG_ANNOUNCE = 5;
	const int IMPACT_WEAK_ANNOUNCE = 6;
	
	const uint8_t ERROR = 11;
	const uint8_t NO_STATUS = 12;
	const uint8_t NO_STATUS_CHANGE = 13; 
	const uint8_t INSPECTION_EXPIRED = 14;
	
	const uint8_t ABOVE_M_FALL_THRESH = 15;
	const uint8_t BELOW_M_FALL_THRESH = 16;
	const uint8_t ABOVE_L_FALL_THRESH  = 17;
	const uint8_t BELOW_L_FALL_THRESH  = 18;
	
	const uint8_t ABOVE_M_IMPACT_THRESH = 19;
	const uint8_t BELOW_M_IMPACT_THRESH = 20;
	const uint8_t ABOVE_H_IMPACT_THRESH  = 21;
	const uint8_t BELOW_H_IMPACT_THRESH  = 22;
	
	const uint8_t NO_CAT= 0;
	const uint8_t FALLING_CAT = 1;
	const uint8_t IMPACTING_CAT = 2;
	
	//const enum Types {FALL_WEAK_CRITERIA, FALL_STRONG_CRITERIA, IMPACT_WEAK_CRITERIA, IMPACT_STRONG_CRITERIA};
	//const enum Cats {NO_CAT, FALLING_CAT, IMPACTING_CAT};
	
	extern bool maybeFalling;
	extern bool likelyFalling;
	
	extern bool maybeImpacting;
	extern bool likelyImpacting;
	
	extern bool inspectionComplete;
	extern bool movementReportable;
	extern int movementType;
	
	extern double aMag;
	
	extern unsigned long tFallStart; 
	
	extern double xVal;
	extern double yVal;
	extern double zVal;
	
	/* this is a problem - dependency on delay between readings */
	extern int numExpiredChecks;
	
	void setCoordVals(sensors_event_t*);
	void checkInspectionComplete();
	
	bool resetMaybeImpacting();
	bool resetMaybeFalling();
	double aMagAverage(double, uint16_t);
	int getCategory(int);
	bool doProcessType(int);
}
/*#######################################
  END FallMonitorContext name
  #######################################**/

/*#######################################
  BEGIN Profile Abstract class
 #######################################**/
class Profile{
 protected:
   bool _isEnabled = false;
   bool _isFinal = false;
   
   uint16_t _updateCount = 0;
   uint16_t _tOffset = 0;
   
 public:     
   bool isEnabled() { return _isEnabled; }
   bool isFinal() { return _isFinal; }

   uint16_t getUpdateCount(){ return _updateCount; }
   uint16_t getTOffset(){ return _tOffset; }
   
   void setTOffset(uint16_t tOffset){ _tOffset = tOffset; }
   void reset(){
       _tOffset = 0;
       _updateCount = 0;
       _isEnabled = false;
       _isFinal = false;
   }
   
   virtual void setEnabled() = 0;
   virtual void setFinal() = 0;
   virtual void update(int, unsigned long) = 0;
};
/*#######################################
  END Profile Abstract class
 #######################################*/


 /*#######################################
   BEGIN AmagProfile class
  #######################################**/
class AmagProfile : public Profile{
  private:
    uint16_t _tFinalElapsed = 0;
    uint16_t _tStartElapsed = 0;
    uint16_t _tDelta = 0;
    
    double _finalVal = 0.0d;
    double _startVal = 0.0d;
    double _finalAve = 0.0d;
    double _startAve = 0.0d;
    double _aMagTotal = 0.0d;
    double _aMagAverage = 0.0d;
    
    void setTDelta(unsigned long);
    void setAmagAverage(int, double, uint16_t);
    
  public:     
    uint16_t getTFinalElapsed(){ return _tFinalElapsed; }
    uint16_t getTStartElapsed(){ return _tStartElapsed; }

    uint16_t getTDelta(){ return _tDelta; }
    
    double getStartVal(){ return _startVal; }
    double getFinalVal(){ return _finalVal; }
    double getStartAve(){ return _startAve; }
    double getFinalAve(){ return _finalAve; }
    double getAmagAverage(){ return _aMagAverage; }
    
    void setEnabled();
    void setFinal();
    void update(int, unsigned long);
    void reset();
    
    AmagProfile() {}
    ~AmagProfile() {}
};
/*#######################################
   END AmagProfile class
  #######################################*/


/*#######################################
   BEGIN ProfileCollection Abstract class
  #######################################*/
class ProfileCollection{
  protected:
    int _type = -1;

  public:
    static const uint8_t ARR_SIZE = 60;
    
    void setType(int type){ _type = type; }
    int getType(){ return _type; }
    
    virtual void reset() = 0;
};
/*#######################################
   END ProfileCollection Abstract class
  #######################################*/


/*#######################################
   BEGIN AmagProfileCollection class
  #######################################*/
class AmagProfileCollection : public ProfileCollection{
  private:
    AmagProfile _aProfiles[ARR_SIZE];

    AmagProfile* _lastEnabledProfile = INVALID_PROFILE;
    AmagProfile* _lastFinalProfile = INVALID_PROFILE;
    AmagProfile* _nextNonFinalProfile = INVALID_PROFILE;
    
    int _nextNonFinalProfileIndex = -1;
    
  public:
    static AmagProfile* INVALID_PROFILE;
	void reset();
	void setFinal(AmagProfile*);
	void setEnabled(AmagProfile*);
	
	AmagProfile* getNextNonFinalProfile(unsigned long);
	AmagProfile* getLastFinalProfile();
	AmagProfile* getLastEnabledProfile();
	AmagProfile* getProfileAt(int);
    
    AmagProfileCollection();
    ~AmagProfileCollection();
};
/*#######################################
   END AmagProfileCollection class
  #######################################*/


/*#######################################
   BEGIN AccelerationAnalyzer class
  #######################################*/
class AccelerationAnalyzer {
  private:
    static const int NUM_PROFILE_COLL_TYPES = 4;
    
    AmagProfileCollection _aProColls[NUM_PROFILE_COLL_TYPES];
    uint8_t getWorkflowStatus(uint16_t, double, int);

  public:
	double lowAmag1 = 100.0d;
	double lowAmag2 = 100.0d;
	double lowAmag3 = 100.0d;
	double lowAmag4 = 100.0d;
	double lowAmag6 = 100.0d;
	double lowAmag7 = 100.0d;
	double lowAmag8 = 100.0d;
	    	
	double highAmag1 = 0.0d;
	double highAmag2 = 0.0d;
	double highAmag3 = 0.0d;
	double highAmag4 = 0.0d;
	double highAmag6 = 0.0d;
	double highAmag7 = 0.0d;
	double highAmag8 = 0.0d;
	
	uint16_t lowAmag1T = 0;
	uint16_t lowAmag2T = 0;
	uint16_t lowAmag3T = 0;
	uint16_t lowAmag4T = 0;
	uint16_t lowAmag6T = 0;
	uint16_t lowAmag7T = 0;
	uint16_t lowAmag8T = 0;
	
	uint16_t highAmag1T = 0;
	uint16_t highAmag2T = 0;
	uint16_t highAmag3T = 0;
	uint16_t highAmag4T = 0;
	uint16_t highAmag6T = 0;
	uint16_t highAmag7T = 0;
	uint16_t highAmag8T = 0;
	
	uint16_t lowAmag1TSpan = 0;
	uint16_t lowAmag2TSpan = 0;
	uint16_t lowAmag3TSpan = 0;
	uint16_t lowAmag4TSpan = 0;
	uint16_t lowAmag6TSpan = 0;
	uint16_t lowAmag7TSpan = 0;
	uint16_t lowAmag8TSpan = 0;
	
	uint16_t highAmag1TSpan = 0;
	uint16_t highAmag2TSpan = 0;
	uint16_t highAmag3TSpan = 0;
	uint16_t highAmag4TSpan = 0;
	uint16_t highAmag6TSpan = 0;
	uint16_t highAmag7TSpan = 0;
	uint16_t highAmag8TSpan = 0;
	
    void resetAmagProfileCollections();
    void processAcceleration(unsigned long);
    void analyzeCollections();
    void resetReport();
    
    AmagProfileCollection* getCollectionAt(int);
    AmagProfileCollection* getCollectionType(int);
    
    AccelerationAnalyzer();
    ~AccelerationAnalyzer();
}; 
/*#######################################
   END AccelerationAnalyzer class
  #######################################*/


/*#######################################
   BEGIN FallManager class
  #######################################*/
class FallManager {
  /* SINGLETON */
  private:
	unsigned long tNow = 0L;
    AccelerationAnalyzer aa;
       
    FallManager();
    FallManager(const FallManager&);    // Don't Implement.
    void operator = (const FallManager&); // Don't implement
    
  public:
    bool isInspectionComplete();
    
    bool isLikelyFalling();
    bool isLikelyImpacting();
    
    bool isMaybeFallingOrImpacting();

    int getMovementType();
    void processAcceleration(unsigned long);
    bool setAmag(sensors_event_t*);
    
    void reset();
    bool doAnalyzeAndReport(sensors_event_t*, unsigned long);
    void reportToBlynk();
    
    static FallManager& getInstance();
        
    ~FallManager() {}
}; 
/*#######################################
   END FallManager class
  #######################################*/