#ifndef CONST_H
#define CONST_H

#define CN_USE_CARDINAL  0 //1 - true, 0 - false
#define CN_HLH_TYPE      0 // 0 - no hlh, 1 - solve lpp by simplex, 2 - greedly take disjoint conflicts
#define CN_USE_DS        0 //1 - true, 0 - false
#define CN_TIMELIMIT     30 // in seconds
#define CN_AGENT_SIZE    sqrt(2.0)/4.0 //radius; only values in range (0; 0.5] are supported
#define CN_CONNECTEDNESS 2 // possible variants 2,3,4,5
#define CN_PRECISION     1e-6
#define CN_FOCAL_WEIGHT  1.0 // experimental function, focal is supported only on the high-level
#define CN_OBSTL         1
#define CN_EPSILON       1e-8
#define CN_INFINITY		 1e+8
#define CN_LOG           "_log"

//XML file tags
#define CNS_TAG_ROOT                "root"
    #define CNS_TAG_ALGORITHM       "algorithm"
    #define CNS_TAG_MAP             "map"
    #define CNS_TAG_AGENTS          "agents"
    #define CNS_TAG_HEIGHT          "height"
    #define CNS_TAG_WIDTH           "width"
    #define CNS_TAG_START_I         "start_i"
    #define CNS_TAG_START_J         "start_j"
    #define CNS_TAG_START_ID        "start_id"
    #define CNS_TAG_GOAL_I          "goal_i"
    #define CNS_TAG_GOAL_J          "goal_j"
    #define CNS_TAG_GOAL_ID         "goal_id"
    #define CNS_TAG_GRID            "grid"
        #define CNS_TAG_ROW         "row"
    #define CNS_TAG_WEIGHT          "weight"
    #define CNS_TAG_OPTIONS         "options"
    #define CNS_TAG_LOGLVL          "loglevel"
    #define CNS_TAG_LOG             "log"
        #define CNS_TAG_MAPFN       "mapfilename"
        #define CNS_TAG_SUM         "summary"
        #define CNS_TAG_PATH        "path"
        #define CNS_TAG_ROW         "row"
        #define CNS_TAG_LPLEVEL     "lplevel"
        #define CNS_TAG_HPLEVEL     "hplevel"
        #define CNS_TAG_AGENT       "agent"
        #define CNS_TAG_LOWLEVEL    "lowlevel"
            #define CNS_TAG_SECTION "section"
            #define CNS_TAG_STEP    "step"
            #define CNS_TAG_OPEN    "open"
            #define CNS_TAG_NODE    "node"
            #define CNS_TAG_CLOSE   "close"
    #define CNS_TAG_ATTR_NUMOFSTEPS     "numberofsteps"
    #define CNS_TAG_ATTR_NODESCREATED   "totalnodescreated"
    #define CNS_TAG_ATTR_LENGTH         "length"
    #define CNS_TAG_ATTR_DURATION       "duration"
    #define CNS_TAG_ATTR_PATHLENGTH     "pathlength"
    #define CNS_TAG_ATTR_TIME           "time"
    #define CNS_TAG_ATTR_X              "x"
    #define CNS_TAG_ATTR_Y              "y"
    #define CNS_TAG_ATTR_NUM            "number"
    #define CNS_TAG_ATTR_F              "f"
    #define CNS_TAG_ATTR_G              "g"
    #define CNS_TAG_ATTR_PARX           "parent_x"
    #define CNS_TAG_ATTR_PARY           "parent_y"
    #define CNS_TAG_ATTR_VALUE          "value"
    #define CNS_TAG_ATTR_SX             "start.x"
    #define CNS_TAG_ATTR_SY             "start.y"
    #define CNS_TAG_ATTR_FX             "finish.x"
    #define CNS_TAG_ATTR_FY             "finish.y"
    #define CNS_TAG_ATTR_FLOWTIME       "flowtime"
    #define CNS_TAG_ATTR_MAKESPAN       "makespan"

#endif // CONST_H
