#pragma once

#define CONST_STRING static constexpr const char *

namespace helm {

   /****************************************************************************
    * structs and types
    */
   struct fsm_state_t {
       std::string name;
       std::string mode;
   };

    struct fsm_transition_t {
        std::string from;
        std::string to;
    };

    struct fsm_state_priority_t{
        std::string name;
        int priority;
    };

    /***************************************************************************
     * XML definitions
     */

    CONST_STRING CONF_CONFIGURATION = "configuration";

    namespace xml {

        CONST_STRING TAG = "Helm";

        struct fsm {
            CONST_STRING TAG = "Fsm";

            struct state {
                CONST_STRING TAG = "State";
                struct ATTRS {
                    CONST_STRING NAME = "name";
                    CONST_STRING MODE = "mode";
                };
            };

            CONST_STRING TAG_STATE = "State";
            struct ATTRS_STATE {
                CONST_STRING NAME = "name";
                CONST_STRING MODE = "mode";
            };
        };

        struct bhvconf {
            CONST_STRING TAG = "BehaviorConfiguration";


            struct behavior {
                CONST_STRING TAG = "Behavior";
                struct ATTRS {
                    CONST_STRING NAME = "name";
                    CONST_STRING PLUGIN = "plugin";
                };

                struct state {
                    CONST_STRING TAG = "State";
                    struct ATTRS {
                        CONST_STRING NAME = "name";
                        CONST_STRING PRIORITY = "priority";
                    };
                };

                struct parameters {
                    CONST_STRING TAG = "Parameters";
                    struct ATTRS {
                        CONST_STRING TYPE = "type";
                        struct OPTIONS_TYPE {
                            CONST_STRING ROS = "ros";
                        };
                    };
                };

            };


        };

    }

}