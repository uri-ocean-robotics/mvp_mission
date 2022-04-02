#pragma once

#include "vector"
#include "string"

#define CONST_STRING static constexpr const char *

namespace helm {

    /***************************************************************************
     * Constants
     */

    static constexpr double DEFAULT_HELM_FREQ = 50;


   /****************************************************************************
    * structs and types
    */
    struct sm_state_t {
        bool initial;
        std::string name;
        std::string mode;
        std::vector<std::string> transitions;
    };

    struct behavior_sm_state_t{
        std::string name;
        int priority;
    };

    struct behavior_component_t{
        std::string name;
        std::string plugin;
        std::map<std::string, int> states;
    };

    struct helm_configuration_t{
        double frequency;
    };

    /***************************************************************************
     * XML definitions
     */

    CONST_STRING CONF_CONFIGURATION = "configuration";

    namespace xml {

        CONST_STRING TAG = "Helm";

        /***********************************************************************
         * Helm XML tags and attributes
         */
         struct helmconf {
             CONST_STRING TAG = "HelmConfiguration";

             struct frequency {
                 CONST_STRING TAG = "frequency";
             };

         };


        /***********************************************************************
         * State machine XML tags and attributes
         */
        struct sm {
            CONST_STRING TAG = "StateMachine";

            struct state {
                CONST_STRING TAG = "State";
                struct ATTRS {
                    CONST_STRING INITIAL = "initial";
                    CONST_STRING NAME = "name";
                    CONST_STRING MODE = "mode";
                };

                struct transition {
                    CONST_STRING TAG = "Transition";
                    struct ATTRS{
                        CONST_STRING TO = "to";
                    };
                };

            };

        };

        /***********************************************************************
         * Behavior configuration XML tags and attributes
         */
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